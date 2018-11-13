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

namespace ORB_SLAM2
{

   MapperClient::MapperClient(cv::FileStorage & settings, ORBVocabulary & vocab, const bool bMonocular) :
      SyncPrint("MapperClient: "),
      mVocab(vocab),
      mbMonocular(bMonocular),
      mInitialized(false),
      mMapperServerObserver(this),
      mContext(2),
      mSocketReq(mContext, ZMQ_REQ)
   {
      mServerAddress.append(settings["Server.Address"]);
      if (0 == mServerAddress.length())
         throw std::exception("Server.Address property is not set or value is not in quotes.");
      Print(string("mServerAddress=") + mServerAddress);

      mPublisherAddress.append(settings["Publisher.Address"]);
      if (0 == mPublisherAddress.length())
         throw std::exception("Publisher.Address property is not set or value is not in quotes.");
      Print(string("mPublisherAddress=") + mPublisherAddress);

      int timeout = settings["Server.Timeout"];
      mSocketReq.setsockopt(ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

      int linger = settings["Server.Linger"];
      mSocketReq.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

      mSocketReq.connect(mServerAddress);

      GreetServer();
   }

   long unsigned MapperClient::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   void MapperClient::Reset()
   {
      // TODO - call server reset
      Print("Begin Server Reset");
      //mServer.Reset();
      Print("End Server Reset");

      // Reset will be received asynchronously from server
      //NotifyReset();

      // Clear Map (this erase MapPoints and KeyFrames)
      Print("Begin Map Reset");
      mMap.Clear();
      Print("End Map Reset");

      mInitialized = false;
      Print("Reset Complete");
   }

   std::vector<KeyFrame *> MapperClient::DetectRelocalizationCandidates(Frame * F)
   {
      // TODO - call server relocalization
      return std::vector<KeyFrame *>();
      //return mServer.DetectRelocalizationCandidates(F);
   }

   bool MapperClient::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperClient::GetPauseRequested()
   {
      //return mServer.GetPauseRequested();
      // TODO - temporary until network synchronization
      return false;
   }

   bool MapperClient::AcceptKeyFrames()
   {
      //return mServer.AcceptKeyFrames();
      // TODO - temporary until network synchronization
      return true;
   }

   void MapperClient::InitializeMono(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2)
   {
      Print("begin InitializeMono");
      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

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

      Print("end LoginTracker");
   }

   void MapperClient::LogoutTracker(unsigned int id)
   {
      Print("begin LogoutTracker");

      zmq::message_t request(sizeof(LogoutTrackerRequest));
      LogoutTrackerRequest * pReqData = request.data<LogoutTrackerRequest>();
      pReqData->serviceId = ServiceId::LOGOUT_TRACKER;
      pReqData->trackerId = id;

      Print("sending LogoutTrackerRequest");
      zmq::message_t reply = RequestReply(request);

      //mServer.LogoutTracker(id);
      Print("end LogoutTracker");
   }

   void MapperClient::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      // TODO - serialize trackerId, pose and send to server
      //mServer.UpdatePose(trackerId, poseTcw);
   }

   vector<cv::Mat> MapperClient::GetTrackerPoses()
   {
      // TODO - pose synchronization between client(s) and server
      //return mServer.GetTrackerPoses();
      return vector<cv::Mat>();
   }

   vector<cv::Mat> MapperClient::GetTrackerPivots()
   {
      // TODO - pivot synchronization between client(s) and server
      //return mServer.GetTrackerPivots();
      return vector<cv::Mat>();
   }

   Map & MapperClient::GetMap()
   {
      // TODO - map synchronization between client(s) and server
      return mMap;
   }

   std::mutex & MapperClient::GetMutexMapUpdate()
   {
      return mMutexMapUpdate;
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
      size_t sizeMsg = sizeof(GeneralRequest) + strlen(HELLO);
      zmq::message_t request(sizeMsg);
      GeneralRequest * pReqData = request.data<GeneralRequest>();
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


   void MapperClient::InitializeMonoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2)
   {
      Print("begin InitializeMonoServer");
      size_t sizeMsg = sizeof(InitializeMonoRequest);
      for (MapPoint * pMP : mapPoints)
      {
         sizeMsg += pMP->GetBufferSize();
      }
      sizeMsg += pKF1->GetBufferSize();
      sizeMsg += pKF2->GetBufferSize();

      zmq::message_t request(sizeMsg);
      InitializeMonoRequest * pReqHead = (InitializeMonoRequest *)request.data();
      pReqHead->serviceId = ServiceId::INITIALIZE_MONO;
      pReqHead->trackerId = trackerId;
      pReqHead->keyFrameId1 = pKF1->GetId();
      pReqHead->keyFrameId2 = pKF2->GetId();
      pReqHead->quantityMapPoints = mapPoints.size();
      char * pData = (char *)(pReqHead + 1);
      for (MapPoint * pMP : mapPoints)
      {
         pData = (char *)pMP->WriteBytes(pData);
      }
      pData = (char *)pKF1->WriteBytes(pData);
      pData = (char *)pKF2->WriteBytes(pData);

      Print("sending InitializeMonoRequest");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      Print("end InitializeMonoServer");
   }

   void MapperClient::InitializeStereoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InitializeStereoServer");
      size_t sizeMsg = sizeof(InitializeStereoRequest);
      for (MapPoint * pMP : mapPoints)
      {
         sizeMsg += pMP->GetBufferSize();
      }
      sizeMsg += pKF->GetBufferSize();

      zmq::message_t request(sizeMsg);
      InitializeStereoRequest * pReqHead = (InitializeStereoRequest *)request.data();
      pReqHead->serviceId = ServiceId::INITIALIZE_STEREO;
      pReqHead->trackerId = trackerId;
      pReqHead->keyFrameId = pKF->GetId();
      pReqHead->quantityMapPoints = mapPoints.size();
      char * pData = (char *)(pReqHead + 1);
      for (MapPoint * pMP : mapPoints)
      {
         pData = (char *)pMP->WriteBytes(pData);
      }
      pData = (char *)pKF->WriteBytes(pData);

      Print("sending InitializeStereoRequest");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      Print("end InitializeStereoServer");
   }

   bool MapperClient::InsertKeyFrameServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InsertKeyFrameServer");

      size_t sizeMsg = sizeof(InsertKeyFrameRequest);
      for (MapPoint * pMP : mapPoints)
      {
         sizeMsg += pMP->GetBufferSize();
      }
      sizeMsg += pKF->GetBufferSize();

      zmq::message_t request(sizeMsg);
      InsertKeyFrameRequest * pReqHead = (InsertKeyFrameRequest *)request.data();
      pReqHead->serviceId = ServiceId::INSERT_KEYFRAME;
      pReqHead->trackerId = trackerId;
      pReqHead->keyFrameId = pKF->GetId();
      pReqHead->quantityMapPoints = mapPoints.size();
      char * pData = (char *)(pReqHead + 1);
      for (MapPoint * pMP : mapPoints)
      {
         pData = (char *)pMP->WriteBytes(pData);
      }
      pData = (char *)pKF->WriteBytes(pData);

      Print("sending InsertKeyFrameRequest");
      zmq::message_t reply = RequestReply(request);
      // map changes are received via the subscriber

      InsertKeyFrameReply * pRepData = reply.data<InsertKeyFrameReply>();

      Print("end InsertKeyFrameServer");
      return pRepData->inserted;
   }

   void MapperClient::MapperServerObserverReset()
   {
      // TODO - reset map
      NotifyReset();
   }

   void MapperClient::MapperServerObserverMapChanged(MapChangeEvent & mce)
   {
      Print("begin MapperServerObserverMapChanged");
      unique_lock<mutex> lock(mMutexMapUpdate);

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
         stringstream ss; ss << "kf->GetId() == " << kf->GetId(); Print(ss);
         KeyFrame * pKF = mMap.GetKeyFrame(kf->GetId());
         if (pKF == NULL)
         {
            Print("pKF == NULL");
            mMap.AddKeyFrame(kf);
         }
         else
         {
            Print("pKF == NULL");
            //pKF->Update(kf);
         }
      }

      stringstream ss3; ss3 << "mce.deletedKeyFrames.size() == " << mce.deletedKeyFrames.size();
      Print(ss3);
      for (unsigned long int id : mce.deletedKeyFrames)
      {
         mMap.EraseKeyFrame(id);
      }

      stringstream ss4; ss4 << "mce.deletedMapPoints.size() == " << mce.deletedMapPoints.size();
      Print(ss4);
      for (unsigned long int id : mce.deletedMapPoints)
      {
         mMap.EraseMapPoint(id);
      }

      mInitialized = true;
      NotifyMapChanged(mce);
      Print("end MapperServerObserverMapChanged");
   }

}