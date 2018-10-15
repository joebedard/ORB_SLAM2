/**
* This file is part of ORB-SLAM2.
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapperClient.h"
#include "Optimizer.h"
#include "Sleep.h"
#include <exception>

namespace ORB_SLAM2
{

   MapperClient::MapperClient(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular) :
      SyncPrint("MapperClient: "),
      mpMap(pMap),
      mpVocab(pVocab),
      mbMonocular(bMonocular),
      mInitialized(false),
      server(new Map(), pVocab, bMonocular),
      mMapperServerObserver(this)
   {
      if (pMap == NULL)
         throw std::exception("pMap must not be NULL");

      if (pVocab == NULL)
         throw std::exception("pVocab must not be NULL");
   }

   long unsigned MapperClient::KeyFramesInMap()
   {
      return mpMap->KeyFramesInMap();
   }

   void MapperClient::Reset()
   {
      // TODO - add an observer to the server
      Print("Begin Server Reset");
      server.Reset();
      Print("End Server Reset");

      NotifyReset();

      // Clear Map (this erase MapPoints and KeyFrames)
      Print("Begin Map Reset");
      mpMap->Clear();
      Print("End Map Reset");

      mInitialized = false;
      Print("Reset Complete");
   }

   std::vector<KeyFrame*> MapperClient::DetectRelocalizationCandidates(Frame* F)
   {
      return server.DetectRelocalizationCandidates(F);
   }

   bool MapperClient::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperClient::GetPauseRequested()
   {
      // temporary until network synchronization
      return false;
   }

   bool MapperClient::AcceptKeyFrames()
   {
      // temporary until network synchronization
      return true;
   }

   void MapperClient::Initialize(unsigned int trackerId, vector<MapPoint*> & mapPoints, vector<KeyFrame*> & keyframes)
   {
      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      server.Initialize(trackerId, mapPoints, keyframes);

      mInitialized = true;
   }

   bool MapperClient::InsertKeyFrame(unsigned int trackerId, vector<MapPoint*> & mapPoints, KeyFrame *pKF)
   {
      // client: serialize KF and MPs and then send to server
      if (server.InsertKeyFrame(trackerId, mapPoints, pKF))
      {
         for (auto pMP : mapPoints)
         {
            mpMap->AddMapPoint(pMP);
         }
         mpMap->AddKeyFrame(pKF);

         return true;
      }
      else
         return false;

   }

   unsigned int MapperClient::LoginTracker(
      unsigned long  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      unsigned long & firstMapPointId,
      unsigned int & mapPointIdSpan,
      const cv::Mat & pivotCalib)
   {
      unique_lock<mutex> lock(mMutexLogin);

      // client: login and get Id values and return them
      return server.LoginTracker(firstKeyFrameId, keyFrameIdSpan, firstMapPointId, mapPointIdSpan, cv::Mat::eye(4, 4, CV_32F));
   }

   void MapperClient::LogoutTracker(unsigned int id)
   {
      unique_lock<mutex> lock(mMutexLogin);

      server.LogoutTracker(id);
   }

   void MapperClient::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      // TODO - serialize trackerId, pose and send to server
      server.UpdatePose(trackerId, poseTcw);
   }

   vector<cv::Mat> MapperClient::GetTrackerPoses()
   {
      // TODO - pose synchronization between client(s) and server
      return server.GetTrackerPoses();
   }

   vector<cv::Mat> MapperClient::GetTrackerPivots()
   {
      // TODO - pivot synchronization between client(s) and server
      return server.GetTrackerPivots();
   }

   Map * MapperClient::GetMap()
   {
      // TODO - map synchronization between client(s) and server
      return mpMap;
   }

   void MapperClient::MapperServerObserverReset()
   {
      // TODO - reset map
      NotifyReset();
   }

   void MapperClient::MapperServerObserverMapChanged(MapChangeEvent & mce)
   {
      // TODO - process map changes
      NotifyMapChanged(mce);
   }

}