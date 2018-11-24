/**
* This file is part of ORB-SLAM2-TEAM.
*
* Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
* For more information see <https://github.com/joebedard/ORB_SLAM2_TEAM>
*
* ORB-SLAM2-TEAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2-TEAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2-TEAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <exception>
#include "MapperClient.h"
#include "Optimizer.h"
#include "Messages.h"
#include "Sleep.h"
#include "Serializer.h"

namespace ORB_SLAM2
{

   MapperClient::MapperClient(cv::FileStorage & settings, ORBVocabulary & vocab, const bool bMonocular) 
      : SyncPrint("MapperClient: ")
      , mVocab(vocab)
      , mKeyFrameDB(vocab)
      , mbMonocular(bMonocular)
      , mInitialized(false)
      , mPauseRequested(false)
      , mAcceptKeyFrames(true)
      , mContext(2)
      , mSocketReq(mContext, ZMQ_REQ)
      , mSocketSub(mContext, ZMQ_SUB)
      , mShouldRun(true)
      , mMessageProc{
         &MapperClient::ReceiveMapReset, 
         &MapperClient::ReceiveMapChange, 
         &MapperClient::ReceivePauseRequested, 
         &MapperClient::ReceiveAcceptKeyFrames,
         &MapperClient::ReceivePivotUpdate,
         &MapperClient::ReceivePoseUpdate}
   {
      // ResetTrackerStatus
      for (int i = 0; i < MAX_TRACKERS; ++i)
      {
         mPivotCalib[i] = cv::Mat::eye(4, 4, CV_32F);
         mPoseTcw[i] = cv::Mat::eye(4, 4, CV_32F);
      }

      mServerAddress.append(settings["Server.Address"]);
      if (0 == mServerAddress.length())
         throw std::exception("Server.Address property is not set or value is not in quotes.");
      Print(string("mServerAddress=") + mServerAddress);

      mPublisherAddress.append(settings["Publisher.Address"]);
      if (0 == mPublisherAddress.length())
         throw std::exception("Publisher.Address property is not set or value is not in quotes.");
      Print(string("mPublisherAddress=") + mPublisherAddress);

      int timeoutServer = settings["Server.Timeout"];
      mSocketReq.setsockopt(ZMQ_RCVTIMEO, &timeoutServer, sizeof(timeoutServer));

      int lingerServer = settings["Server.Linger"];
      mSocketReq.setsockopt(ZMQ_LINGER, &lingerServer, sizeof(lingerServer));

      mSocketReq.connect(mServerAddress);

      int timeoutPub = settings["Publisher.Timeout"];
      mSocketReq.setsockopt(ZMQ_RCVTIMEO, &timeoutPub, sizeof(timeoutPub));

      int lingerPub = settings["Publisher.Linger"];
      mSocketReq.setsockopt(ZMQ_LINGER, &lingerPub, sizeof(lingerPub));

      mSocketSub.connect(mPublisherAddress);
      mSocketSub.setsockopt<unsigned int>(ZMQ_SUBSCRIBE, -1); // -1 is for broadcast messages

      //Initialize and start the Subscriber thread
      mThreadSub = new thread(&ORB_SLAM2::MapperClient::RunSubscriber, this);

      GreetServer();
   }

   MapperClient::~MapperClient()
   {
      if (mThreadSub)
      {
         mShouldRun = false;
         mThreadSub->join();
         delete mThreadSub;
      }
   }

   long unsigned MapperClient::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   void MapperClient::Reset()
   {
      Print("begin Reset");
      unique_lock<mutex> lock(mMutexMapUpdate);

      zmq::message_t request(sizeof(GeneralRequest));
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::RESET;

      zmq::message_t reply = RequestReply(request);

      // Reset will be received asynchronously from server
      Print("end Reset");
   }

   std::vector<KeyFrame *> MapperClient::DetectRelocalizationCandidates(Frame * F)
   {
      return mKeyFrameDB.DetectRelocalizationCandidates(F);
   }

   bool MapperClient::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperClient::GetPauseRequested()
   {
      unique_lock<mutex> lock(mMutexPause);
      return mPauseRequested;
   }

   bool MapperClient::AcceptKeyFrames()
   {
      unique_lock<mutex> lock(mMutexAccept);
      return mAcceptKeyFrames;
   }

   void MapperClient::InitializeMono(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2)
   {
      Print("begin InitializeMono");
      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      pKF1->ComputeBoW(mVocab);
      pKF2->ComputeBoW(mVocab);

      // stereo and RGBD modes will create MapPoints
      for (auto it : mapPoints)
      {
         mMap.AddMapPoint(it);
      }

      // is this needed for tracking client?
      mMap.mvpKeyFrameOrigins.push_back(pKF1);

      // Insert KeyFrame in the map
      mMap.AddKeyFrame(pKF1);
      mMap.AddKeyFrame(pKF2);

      InitializeMonoServer(trackerId, mapPoints, pKF1, pKF2);

      mInitialized = true;
      Print("end InitializeMono");
   }

   void MapperClient::InitializeStereo(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InitializeStereo");
      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      pKF->ComputeBoW(mVocab);

      // stereo and RGBD modes will create MapPoints
      for (auto it : mapPoints)
      {
         mMap.AddMapPoint(it);
      }

      // is this needed for tracking client?
      mMap.mvpKeyFrameOrigins.push_back(pKF);

      // Insert KeyFrame in the map
      mMap.AddKeyFrame(pKF);

      InitializeStereoServer(trackerId, mapPoints, pKF);

      mInitialized = true;
      Print("end InitializeStereo");
   }

   bool MapperClient::InsertKeyFrame(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InsertKeyFrame");
      // client: serialize KF and MPs and then send to server
      if (InsertKeyFrameServer(trackerId, mapPoints, pKF))
      {
         pKF->ComputeBoW(mVocab);
         // add points and keyframes to allow for map synchronization with the server
         // which will happen after the Tracking client unlocks mMutexMapUpdate
         for (MapPoint * pMP : mapPoints)
         {
            mMap.AddMapPoint(pMP);
         }
         mMap.AddKeyFrame(pKF);

         Print("end InsertKeyFrame 1");
         return true;
      }
      else
      {
         Print("end InsertKeyFrame 2");
         return false;
      }

   }

   void MapperClient::LoginTracker(
      const cv::Mat & pivotCalib,
      unsigned int & trackerId,
      unsigned long  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      unsigned long & firstMapPointId,
      unsigned int & mapPointIdSpan)
   {
      Print("begin LoginTracker");

      LoginTrackerServer(pivotCalib, trackerId, firstKeyFrameId, keyFrameIdSpan, firstMapPointId, mapPointIdSpan);

      {
         unique_lock<mutex> lock(mMutexSocketSub);
         mSocketSub.setsockopt<unsigned int>(ZMQ_SUBSCRIBE, trackerId);
      }

      if (trackerId != 0)
      {
         GetMapFromServer(trackerId);
      }

      Print("end LoginTracker");
   }

   void MapperClient::LogoutTracker(unsigned int id)
   {
      Print("begin LogoutTracker");

      zmq::message_t request(sizeof(GeneralRequest));
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::LOGOUT_TRACKER;
      pReqData->trackerId = id;

      Print("sending LogoutTrackerRequest");
      zmq::message_t reply = RequestReply(request);

      Print("end LogoutTracker");
   }

   void MapperClient::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);
      mPoseTcw[trackerId] = poseTcw.clone();
      UpdatePoseServer(trackerId, poseTcw);
   }

   vector<cv::Mat> MapperClient::GetTrackerPoses()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (int i = 0; i < MAX_TRACKERS; i++)
      {
         poses.push_back(mPoseTcw[i].clone());
      }
      return poses;
   }

   vector<cv::Mat> MapperClient::GetTrackerPivots()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (int i = 0; i < MAX_TRACKERS; i++)
      {
         poses.push_back(mPivotCalib[i].clone());
      }
      return poses;
   }

   Map & MapperClient::GetMap()
   {
      return mMap;
   }

   std::mutex & MapperClient::GetMutexMapUpdate()
   {
      return mMutexMapUpdate;
   }

   void MapperClient::ReceiveMapReset(zmq::message_t & message)
   {
      Print("begin ReceiveMapReset");
      unique_lock<mutex> lock(mMutexMapUpdate);
      NotifyMapReset();

      // Clear BoW Database
      Print("Begin Database Reset");
      mKeyFrameDB.clear();
      Print("End Database Reset");

      // Clear Map (this erase MapPoints and KeyFrames)
      Print("Begin Map Reset");
      mMap.Clear();
      Print("End Map Reset");

      mInitialized = false;
      Print("Reset Complete");
      Print("end ReceiveMapReset");
   }

   void MapperClient::ReceiveMapChange(zmq::message_t & message)
   {
      Print("begin ReceiveMapChange");

      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      MapChangeEvent mce;
      unique_lock<mutex> lock(mMutexMapUpdate);
      mce.ReadBytes(pMsgData + 1, mMap);

      // be careful - process map changes in the best order

      stringstream ss1; ss1 << "mce.updatedMapPoints.size() == " << mce.updatedMapPoints.size();
      Print(ss1);
      for (MapPoint * mp : mce.updatedMapPoints)
      {
         MapPoint * pMP = mMap.GetMapPoint(mp->GetId());
         if (pMP == NULL)
         {
            mMap.AddMapPoint(mp);
         }
         else
         {
            //pMP->Update(mp);
         }
      }

      stringstream ss2; ss2 << "mce.updatedKeyFrames.size() == " << mce.updatedKeyFrames.size();
      Print(ss2);
      for (KeyFrame * kf : mce.updatedKeyFrames)
      {
         //stringstream ss; ss << "kf->GetId() == " << kf->GetId(); Print(ss);
         KeyFrame * pKF = mMap.GetKeyFrame(kf->GetId());
         if (pKF == NULL)
         {
            Print("pKF == NULL");
            mMap.AddKeyFrame(kf);
            mKeyFrameDB.add(kf);
         }
         else
         {
            //pKF->Update(kf);
         }
      }

      stringstream ss3; ss3 << "mce.deletedKeyFrames.size() == " << mce.deletedKeyFrames.size();
      Print(ss3);
      for (unsigned long int id : mce.deletedKeyFrames)
      {
         KeyFrame * pKF = mMap.GetKeyFrame(id);
         if (pKF)
         {
            mMap.EraseKeyFrame(id);
            mKeyFrameDB.erase(pKF);
         }
      }

      stringstream ss4; ss4 << "mce.deletedMapPoints.size() == " << mce.deletedMapPoints.size();
      Print(ss4);
      for (unsigned long int id : mce.deletedMapPoints)
      {
         mMap.EraseMapPoint(id);
      }

      mInitialized = true;
      NotifyMapChanged(mce);
      Print("end ReceiveMapChange");
   }

   void MapperClient::ReceivePauseRequested(zmq::message_t & message)
   {
      Print("begin ReceivePauseRequested");
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      unique_lock<mutex> lock(mMutexPause);
      mPauseRequested = *(bool *)(pMsgData + 1);
      Print("end ReceivePauseRequested");
   }

   void MapperClient::ReceiveAcceptKeyFrames(zmq::message_t & message)
   {
      Print("begin ReceiveAcceptKeyFrames");
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      unique_lock<mutex> lock(mMutexAccept);
      mAcceptKeyFrames = *(bool *)(pMsgData + 1);
      Print("end ReceiveAcceptKeyFrames");
   }

   void MapperClient::ReceivePivotUpdate(zmq::message_t & message)
   {
      Print("begin ReceivePivotUpdate");
      UpdateTrackerMessage * pMsgData = message.data<UpdateTrackerMessage>();
      unique_lock<mutex> lock(mMutexTrackerStatus);
      cv::Mat pivotCalib;
      void * pData = Serializer::ReadMatrix(pMsgData + 1, pivotCalib);
      mPivotCalib[pMsgData->trackerId] = pivotCalib.clone();
      Print("end ReceivePivotUpdate");
   }

   void MapperClient::ReceivePoseUpdate(zmq::message_t & message)
   {
      Print("begin ReceivePoseUpdate");
      UpdateTrackerMessage * pMsgData = message.data<UpdateTrackerMessage>();
      unique_lock<mutex> lock(mMutexTrackerStatus);
      cv::Mat poseTcw;
      void * pData = Serializer::ReadMatrix(pMsgData + 1, poseTcw);
      stringstream ss; ss << poseTcw; Print(ss);
      mPoseTcw[pMsgData->trackerId] = poseTcw.clone();
      Print("end ReceivePoseUpdate");
   }

   void MapperClient::RunSubscriber() try
   {
      Print("begin RunSubscriber");
      zmq::message_t message;
      while (mShouldRun)
      {
         bool received = false;
         {
            unique_lock<mutex> lock(mMutexSocketSub);
            received = mSocketSub.recv(&message, ZMQ_NOBLOCK);
         }
         if (received)
         {
            GeneralMessage * pReqData = message.data<GeneralMessage>();
            try 
            {
               if (pReqData->messageId < MessageId::quantityMessageId)
               {
                  (this->*mMessageProc[pReqData->messageId])(message);
               }
               else
               {
                  stringstream ss; 
                  ss << "RunSubscriber received an unknown message with id=" << pReqData->messageId;
                  Print(ss);
               }
            } 
            catch (std::exception & e)
            {
               stringstream ss;
               ss << "RunSubscriber exception in message procedure id="  << pReqData->messageId;
               ss << ": " << e.what();
               Print(ss);
            }
         }
         else
         {
            sleep(1000);
         }
      }
      Print("end RunSubscriber");
   }
   catch (zmq::error_t & e)
   {
      Print(string("RunSubscriber: error_t: ") + e.what());
   }
   catch (const std::exception & e)
   {
      Print(string("RunSubscriber: exception: ") + e.what());
   }
   catch (...)
   {
      Print("RunSubscriber: an exception was not caught");
   }

   zmq::message_t MapperClient::RequestReply(zmq::message_t & request)
   {
      Print("begin RequestReply");

      mSocketReq.send(request);
      zmq::message_t reply;
      mSocketReq.recv(&reply);
      GeneralReply * pReplyData = reply.data<GeneralReply>();
      switch (pReplyData->replyCode)
      {
      case ReplyCode::SUCCEEDED:
         Print("end RequestReply");
         return reply;

      case ReplyCode::FAILED:
         throw exception(string("server failed: ").append(pReplyData->message).c_str());

      case ReplyCode::UNKNOWN_SERVICE:
         throw exception("the server does not support the requested service");

      default:
         throw exception("received an unknown reply code");
      }

   }

   void MapperClient::GreetServer()
   {
      const char * HELLO = "Hello";
      size_t sizeMsg = sizeof(GreetRequest) + strlen(HELLO);
      zmq::message_t request(sizeMsg);
      GreetRequest * pReqData = request.data<GreetRequest>();
      pReqData->serviceId = ServiceId::HELLO;
      strcpy(pReqData->message, HELLO);

      Print("Sending Hello");
      zmq::message_t reply = RequestReply(request);
      
      GeneralReply * pRepData = reply.data<GeneralReply>();
      if (0 == strcmp(pRepData->message, "World"))
      {
         Print("Received World");
      }
      else
      {
         Print("unable to greet server");
      }
   }

   void MapperClient::LoginTrackerServer(
      const cv::Mat & pivotCalib,
      unsigned int & trackerId,
      id_type  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      id_type & firstMapPointId,
      unsigned int & mapPointIdSpan)
   {
      Print("begin LoginTrackerServer");

      zmq::message_t request(sizeof(LoginTrackerRequest));
      LoginTrackerRequest * pReqData = request.data<LoginTrackerRequest>();
      pReqData->serviceId = ServiceId::LOGIN_TRACKER;
      pReqData->pivotCalib[0] = pivotCalib.at<float>(0, 0);
      pReqData->pivotCalib[1] = pivotCalib.at<float>(0, 1);
      pReqData->pivotCalib[2] = pivotCalib.at<float>(0, 2);
      pReqData->pivotCalib[3] = pivotCalib.at<float>(0, 3);
      pReqData->pivotCalib[4] = pivotCalib.at<float>(1, 0);
      pReqData->pivotCalib[5] = pivotCalib.at<float>(1, 1);
      pReqData->pivotCalib[6] = pivotCalib.at<float>(1, 2);
      pReqData->pivotCalib[7] = pivotCalib.at<float>(1, 3);
      pReqData->pivotCalib[8] = pivotCalib.at<float>(2, 0);
      pReqData->pivotCalib[9] = pivotCalib.at<float>(2, 1);
      pReqData->pivotCalib[10] = pivotCalib.at<float>(2, 2);
      pReqData->pivotCalib[11] = pivotCalib.at<float>(2, 3);
      pReqData->pivotCalib[12] = pivotCalib.at<float>(3, 0);
      pReqData->pivotCalib[13] = pivotCalib.at<float>(3, 1);
      pReqData->pivotCalib[14] = pivotCalib.at<float>(3, 2);
      pReqData->pivotCalib[15] = pivotCalib.at<float>(3, 3);

      // login and get Id values and return them
      Print("sending LoginTrackerRequest");
      zmq::message_t reply = RequestReply(request);
      LoginTrackerReply * pRepData = reply.data<LoginTrackerReply>();
      trackerId = pRepData->trackerId;
      firstKeyFrameId = pRepData->firstKeyFrameId;
      keyFrameIdSpan = pRepData->keyFrameIdSpan;
      firstMapPointId = pRepData->firstMapPointId;
      mapPointIdSpan = pRepData->mapPointIdSpan;

      Print("end LoginTrackerServer");
   }

   void MapperClient::GetMapFromServer(const unsigned int trackerId)
   {
      Print("begin GetMapFromServer");

      zmq::message_t request(sizeof(GeneralRequest));
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::GET_MAP;
      pReqData->trackerId = trackerId;

      Print("sending GetMapRequest");
      zmq::message_t reply = RequestReply(request);

      Print("end GetMapFromServer");
   }

   void MapperClient::UpdatePoseServer(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      Print("begin UpdatePoseServer");
      size_t sizeMsg = sizeof(GeneralRequest);
      sizeMsg += Serializer::GetMatBufferSize(poseTcw);

      zmq::message_t request(sizeMsg);
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::UPDATE_POSE;
      pReqData->trackerId = trackerId;
      Serializer::WriteMatrix(pReqData + 1, poseTcw);

      Print("sending UpdatePose");
      zmq::message_t reply = RequestReply(request);

      Print("end UpdatePoseServer");
   }

   void MapperClient::InitializeMonoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2)
   {
      Print("begin InitializeMonoServer");
      size_t sizeMsg = sizeof(GeneralRequest);
      sizeMsg += pKF1->GetBufferSize();
      sizeMsg += pKF2->GetBufferSize();
      sizeMsg += MapPoint::GetVectorBufferSize(mapPoints);

      zmq::message_t request(sizeMsg);
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::INITIALIZE_MONO;
      pReqData->trackerId = trackerId;
      void * pData = pReqData + 1;
      pData = pKF1->WriteBytes(pData);
      pData = pKF2->WriteBytes(pData);
      pData = MapPoint::WriteVector(pData, mapPoints);

      Print("sending InitializeMonoRequest");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      Print("end InitializeMonoServer");
   }

   void MapperClient::InitializeStereoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InitializeStereoServer");
      size_t sizeMsg = sizeof(GeneralRequest);
      sizeMsg += pKF->GetBufferSize();
      sizeMsg += MapPoint::GetVectorBufferSize(mapPoints);

      Print("create InitializeStereoServer");
      zmq::message_t request(sizeMsg);
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::INITIALIZE_STEREO;
      pReqData->trackerId = trackerId;
      void * pData = pReqData + 1;
      pData = pKF->WriteBytes(pData);
      pData = MapPoint::WriteVector(pData, mapPoints);

      Print("sending InitializeStereoServer");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      Print("end InitializeStereoServer");
   }

   bool MapperClient::InsertKeyFrameServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InsertKeyFrameServer");

      size_t sizeMsg = sizeof(GeneralRequest);
      sizeMsg += pKF->GetBufferSize();
      sizeMsg += MapPoint::GetVectorBufferSize(mapPoints);

      zmq::message_t request(sizeMsg);
      GeneralRequest * pReqData = request.data<GeneralRequest>();
      pReqData->serviceId = ServiceId::INSERT_KEYFRAME;
      pReqData->trackerId = trackerId;
      void * pData = pReqData + 1;
      pData = pKF->WriteBytes(pData);
      pData = MapPoint::WriteVector(pData, mapPoints);

      Print("sending InsertKeyFrameRequest");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      InsertKeyFrameReply * pRepData = reply.data<InsertKeyFrameReply>();

      Print("end InsertKeyFrameServer");
      return pRepData->inserted;
   }

}