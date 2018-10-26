/**
* This file is part of ORB-SLAM2-NET.
*
* Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
* For more information see <https://github.com/joebedard/ORB_SLAM2_NET>
*
* ORB-SLAM2-NET is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2-NET is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2-NET. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapperClient.h"
#include "Optimizer.h"
#include "Sleep.h"
#include <exception>

namespace ORB_SLAM2
{

   MapperClient::MapperClient(cv::FileStorage & settings, ORBVocabulary & vocab, const bool bMonocular) :
      SyncPrint("MapperClient: "),
      mServer(vocab, bMonocular),
      mVocab(vocab),
      mbMonocular(bMonocular),
      mInitialized(false),
      mMapperServerObserver(this),
      mContext(1),
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

      mServer.AddObserver(&mMapperServerObserver);

      GreetServer();
   }

   long unsigned MapperClient::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   void MapperClient::Reset()
   {
      Print("Begin Server Reset");
      mServer.Reset();
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

   std::vector<KeyFrame*> MapperClient::DetectRelocalizationCandidates(Frame* F)
   {
      return mServer.DetectRelocalizationCandidates(F);
   }

   bool MapperClient::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperClient::GetPauseRequested()
   {
      return mServer.GetPauseRequested();
      // TODO - temporary until network synchronization
      //return false;
   }

   bool MapperClient::AcceptKeyFrames()
   {
      return mServer.AcceptKeyFrames();
      // TODO - temporary until network synchronization
      //return true;
   }

   void MapperClient::Initialize(unsigned int trackerId, vector<MapPoint*> & mapPoints, vector<KeyFrame*> & keyframes)
   {
      Print("begin Initialize");
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
      mMap.mvpKeyFrameOrigins.push_back(keyframes[0]);

      for (auto it : keyframes)
      {
         // Insert KeyFrame in the map
         mMap.AddKeyFrame(it);
      }

      mServer.Initialize(trackerId, mapPoints, keyframes);

      mInitialized = true;
      Print("end Initialize");
   }

   bool MapperClient::InsertKeyFrame(unsigned int trackerId, vector<MapPoint*> & mapPoints, KeyFrame *pKF)
   {
      Print("begin InsertKeyFrame");
      // client: serialize KF and MPs and then send to server
      if (mServer.InsertKeyFrame(trackerId, mapPoints, pKF))
      {
         // add points and keyframes to allow for for map synchronization with the server
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

   unsigned int MapperClient::LoginTracker(
      unsigned long  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      unsigned long & firstMapPointId,
      unsigned int & mapPointIdSpan,
      const cv::Mat & pivotCalib)
   {
      Print("begin LoginTracker");
      unique_lock<mutex> lock(mMutexLogin);

      // client: login and get Id values and return them
      int id = mServer.LoginTracker(firstKeyFrameId, keyFrameIdSpan, firstMapPointId, mapPointIdSpan, pivotCalib);
      Print("end LoginTracker");
      return id;
   }

   void MapperClient::LogoutTracker(unsigned int id)
   {
      unique_lock<mutex> lock(mMutexLogin);

      mServer.LogoutTracker(id);
   }

   void MapperClient::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      // TODO - serialize trackerId, pose and send to server
      mServer.UpdatePose(trackerId, poseTcw);
   }

   vector<cv::Mat> MapperClient::GetTrackerPoses()
   {
      // TODO - pose synchronization between client(s) and server
      return mServer.GetTrackerPoses();
   }

   vector<cv::Mat> MapperClient::GetTrackerPivots()
   {
      // TODO - pivot synchronization between client(s) and server
      return mServer.GetTrackerPivots();
   }

   Map & MapperClient::GetMap()
   {
      // TODO - map synchronization between client(s) and server
      return mMap;
   }

   std::mutex & MapperClient::GetMutexMapUpdate()
   {
      // TODO - replace with client mutex
      return mServer.GetMutexMapUpdate();
   }

   zmq::message_t MapperClient::RequestReply(zmq::message_t & request)
   {
      mSocketReq.send(request);
      zmq::message_t reply;
      mSocketReq.recv(&reply);
      ReplyCode * pReplyCode = (ReplyCode *)reply.data();
      char * pStr = (char *)(pReplyCode + 1);
      switch (*pReplyCode)
      {
      case ReplyCode::Succeeded:
         return reply;

      case ReplyCode::Failed:
         throw exception(string("server failed: ").append(pStr).c_str());

      case ReplyCode::UnknownService:
         throw exception("the server does not support the requested service");

      default:
         throw exception("received an unknown reply code");
      }
   }

   void MapperClient::GreetServer()
   {
      const char * HELLO = "Hello";
      size_t sizeMsg = sizeof(ServiceID) + sizeof(char) * (strlen(HELLO) + 1);
      zmq::message_t request(sizeMsg);
      ServiceID * pServiceID = (ServiceID *)request.data();
      *pServiceID = ServiceID::Hello;
      char * str = (char *)(pServiceID + 1);
      strcpy(str, HELLO);
      Print("Sending Hello");
      zmq::message_t reply = RequestReply(request);
      ReplyCode * pReplyCode = (ReplyCode *)reply.data();
      char * pStr = (char *)(pReplyCode + 1);
      if (0 == strcmp(pStr, "World"))
      {
         Print("Received World");
      }
      else
      {
         Print("unable to greet server");
      }
   }

   void MapperClient::MapperServerObserverReset()
   {
      // TODO - reset map
      NotifyReset();
   }

   void MapperClient::MapperServerObserverMapChanged(MapChangeEvent & mce)
   {
      Print("begin MapperServerObserverMapChanged");
      unique_lock<mutex> lock(mServer.GetMutexMapUpdate());

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