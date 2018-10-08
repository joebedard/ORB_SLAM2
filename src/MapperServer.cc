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

#include "MapperServer.h"
#include "Optimizer.h"
#include "Sleep.h"
#include <exception>

namespace ORB_SLAM2
{

   MapperServer::MapperServer(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular) :
        SyncPrint("MapperServer: "), 
        mpMap(pMap),
        mpVocab(pVocab), 
        mbMonocular(bMonocular), 
        mInitialized(false),
        mLocalMappingObserver(this),
        mLoopClosingObserver(this)
   {
      if (pMap == NULL)
         throw std::exception("pMap must not be NULL");

      if (pVocab == NULL)
         throw std::exception("pVocab must not be NULL");

      mpKeyFrameDB = new KeyFrameDatabase(*pVocab);

      ResetTrackerStatus();

      //Initialize and start the Local Mapping thread
      mpLocalMapper = new LocalMapping(mpMap, mpKeyFrameDB, mbMonocular, FIRST_MAPPOINT_ID_LOCALMAPPER, MAPPOINT_ID_SPAN);
      mpLocalMapper->AddObserver(&mLocalMappingObserver);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

      //Initialize and start the Loop Closing thread
      mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDB, mpVocab, !mbMonocular);
      mpLoopCloser->AddObserver(&mLoopClosingObserver);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

      mpLocalMapper->SetLoopCloser(mpLoopCloser);
      mpLoopCloser->SetLocalMapper(mpLocalMapper);
   }

   long unsigned MapperServer::KeyFramesInMap()
   {
      return mpMap->KeyFramesInMap();
   }

   void MapperServer::Reset()
   {
      ResetTrackerStatus();

      // Reset Local Mapping
      Print("Begin Local Mapper Reset");
      mpLocalMapper->RequestReset();
      Print("End Local Mapper Reset");

      // Reset Loop Closing
      Print("Begin Loop Closing Reset");
      mpLoopCloser->RequestReset();
      Print("End Loop Closing Reset");

      NotifyReset();

      // Clear BoW Database
      Print("Begin Database Reset");
      mpKeyFrameDB->clear();
      Print("End Database Reset");

      // Clear Map (this erase MapPoints and KeyFrames)
      Print("Begin Map Reset");
      mpMap->Clear();
      Print("End Map Reset");

      mInitialized = false;
      Print("Reset Complete");
   }

   std::vector<KeyFrame*> MapperServer::DetectRelocalizationCandidates(Frame* F)
   {
      return mpKeyFrameDB->DetectRelocalizationCandidates(F);
   }
      
   bool MapperServer::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperServer::GetPauseRequested()
   {
      return mpLocalMapper->PauseRequested();
   }
   
   bool MapperServer::AcceptKeyFrames()
   {
      return mpLocalMapper->AcceptKeyFrames();
   }
   
   void MapperServer::Shutdown()
   {
      mpLocalMapper->RequestFinish();
      mpLoopCloser->RequestFinish();

      // Wait until all thread have effectively stopped
      while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
      {
         sleep(5000);
      }
   }

   void MapperServer::Initialize(unsigned int trackerId, vector<MapPoint*> & mapPoints, vector<KeyFrame*> & keyframes)
   {
      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      ValidateTracker(trackerId);

      // stereo and RGBD modes will create MapPoints
      for (auto it : mapPoints)
      {
          mpMap->AddMapPoint(it);
      }

      mpMap->mvpKeyFrameOrigins.push_back(keyframes[0]);

      for (auto it : keyframes)
      {
          // Insert KeyFrame in the map
          mpMap->AddKeyFrame(it);
      }

      // all points are already added to the map
      vector<MapPoint *> noAdditionalPoints;

      for (auto it : keyframes)
      {
          if (!InsertKeyFrame(trackerId, noAdditionalPoints, it))
          {
             mpMap->Clear();
             throw exception("Unable to InsertKeyFrame during Initialize.");
          }
      }

      // stereo and RGBD modes will create MapPoints
      UpdateTrackerIds(trackerId, mapPoints);

      mInitialized = true;
   }

   bool MapperServer::InsertKeyFrame(unsigned int trackerId, vector<MapPoint*> & mapPoints, KeyFrame *pKF)
   {
      ValidateTracker(trackerId);

      if (mpLocalMapper->InsertKeyFrame(mapPoints, pKF))
      {
         // update TrackerStatus array with next KeyFrameId and MapPointId
         assert((pKF->GetId() - trackerId) % KEYFRAME_ID_SPAN == 0);
         if (mTrackers[trackerId].nextKeyFrameId <= pKF->GetId())
         {
            mTrackers[trackerId].nextKeyFrameId = pKF->GetId() + KEYFRAME_ID_SPAN;
         }

         // stereo and RGBD modes will create MapPoints
         UpdateTrackerIds(trackerId, mapPoints);

         return true;
      }
      else
         return false;
   }

   void MapperServer::UpdateTrackerIds(unsigned int trackerId, vector<MapPoint *> mapPoints)
   {
       for (auto pMP : mapPoints)
       {
           assert((pMP->GetId() - trackerId) % MAPPOINT_ID_SPAN == 0);
           if (mTrackers[trackerId].nextMapPointId <= pMP->GetId())
           {
               mTrackers[trackerId].nextMapPointId = pMP->GetId() + MAPPOINT_ID_SPAN;
           }
       }
   }

   unsigned int MapperServer::LoginTracker(
        unsigned long  & firstKeyFrameId, 
        unsigned int & keyFrameIdSpan, 
        unsigned long & firstMapPointId, 
        unsigned int & mapPointIdSpan,
        const cv::Mat & pivotCalib)
   {
      unique_lock<mutex> lock(mMutexLogin);

      unsigned int id;
      for (id = 0; id < MAX_TRACKERS; ++id)
      {
         if (!mTrackers[id].connected)
         {
            mTrackers[id].connected = true;
            mTrackers[id].pivotCalib = pivotCalib;
            break;
         }
      }

      if (id >= MAX_TRACKERS)
         throw std::exception("Maximum number of trackers reached. Additional trackers are not supported.");

      firstKeyFrameId = mTrackers[id].nextKeyFrameId;
      keyFrameIdSpan = KEYFRAME_ID_SPAN;
      firstMapPointId = mTrackers[id].nextMapPointId;
      mapPointIdSpan = MAPPOINT_ID_SPAN;

      return id;
   }

   void MapperServer::LogoutTracker(unsigned int id)
   {
      mTrackers[id].connected = false;
   }

   void MapperServer::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      // TODO - complete this function
   }

   Map * MapperServer::GetMap()
   {
       return mpMap;
   }

   void MapperServer::ResetTrackerStatus()
   {
      unique_lock<mutex> lock(mMutexLogin);

      for (int i = 0; i < MAX_TRACKERS; ++i)
      {
         mTrackers[i].connected = false;
         mTrackers[i].nextKeyFrameId = i;
         mTrackers[i].nextMapPointId = i;
      }
   }

   void MapperServer::ValidateTracker(unsigned int trackerId)
   {
       if (!mTrackers[trackerId].connected)
           throw exception(string("Tracker is not logged in! Id=").append(to_string(trackerId)).c_str());
   }

}