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

#include "MapperServer.h"
#include "Optimizer.h"
#include "Sleep.h"
#include <exception>

namespace ORB_SLAM2
{

   MapperServer::MapperServer(ORBVocabulary & vocab, const bool bMonocular) :
      SyncPrint("MapperServer: "),
      mVocab(vocab),
      mbMonocular(bMonocular),
      mKeyFrameDB(vocab),
      mInitialized(false),
      mLocalMapper(mMap, mMutexMapUpdate, mKeyFrameDB, mVocab, bMonocular, FIRST_MAPPOINT_ID_LOCALMAPPER, MAPPOINT_ID_SPAN),
      mLoopCloser(mMap, mMutexMapUpdate, mKeyFrameDB, mVocab, !bMonocular),
      mLocalMappingObserver(this),
      mLoopClosingObserver(this)
   {
      ResetTrackerStatus();

      //Initialize and start the Local Mapping thread
      mLocalMapper.AddObserver(&mLocalMappingObserver);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, &mLocalMapper);

      //Initialize and start the Loop Closing thread
      mLoopCloser.AddObserver(&mLoopClosingObserver);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, &mLoopCloser);

      mLocalMapper.SetLoopCloser(&mLoopCloser);
      mLoopCloser.SetLocalMapper(&mLocalMapper);
   }

   MapperServer::~MapperServer()
   {
      mLocalMapper.RequestFinish();
      mLoopCloser.RequestFinish();

      mptLocalMapping->join();
      mptLoopClosing->join();

      delete mptLocalMapping;
      delete mptLoopClosing;
   }

   long unsigned MapperServer::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   void MapperServer::Reset()
   {
      ResetTrackerStatus();

      // Reset Local Mapping
      Print("Begin Local Mapper Reset");
      mLocalMapper.RequestReset();
      Print("End Local Mapper Reset");

      // Reset Loop Closing
      Print("Begin Loop Closing Reset");
      mLoopCloser.RequestReset();
      Print("End Loop Closing Reset");

      NotifyReset();

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
   }

   std::vector<KeyFrame *> MapperServer::DetectRelocalizationCandidates(Frame * F)
   {
      return mKeyFrameDB.DetectRelocalizationCandidates(F);
   }

   bool MapperServer::GetInitialized()
   {
      return mInitialized;
   }

   bool MapperServer::GetPauseRequested()
   {
      return mLocalMapper.PauseRequested();
   }

   bool MapperServer::AcceptKeyFrames()
   {
      return mLocalMapper.AcceptKeyFrames();
   }

   void MapperServer::InitializeMono(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2)
   {
      Print("begin InitializeMono");

      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      if (!mbMonocular)
         throw exception("Monocular initialize is not allowed on a stereo map.");

      ValidateTracker(trackerId);

      for (MapPoint * pMP : mapPoints)
      {
         mMap.AddMapPoint(pMP);
      }

      // Insert KeyFrame in the map
      mMap.mvpKeyFrameOrigins.push_back(pKF1);
      mMap.AddKeyFrame(pKF1);
      mMap.AddKeyFrame(pKF2);

      if (mLocalMapper.InsertKeyFrame(pKF1) && mLocalMapper.InsertKeyFrame(pKF2))
      {
         UpdateTrackerStatus(trackerId, mapPoints);
         UpdateTrackerStatus(trackerId, pKF1);
         UpdateTrackerStatus(trackerId, pKF2);
         mInitialized = true;
      }
      else
      {
         mMap.Clear();
         throw exception("Unable to InsertKeyFrame during InitializeMono.");
      }
      Print("end InitializeMono");
   }

   void MapperServer::InitializeStereo(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      Print("begin InitializeStereo");

      if (mInitialized)
         throw exception("The mapper may only be initialized once.");

      if (trackerId != 0)
         throw exception("Only the first Tracker (id=0) may initialize the map.");

      if (mbMonocular)
         throw exception("Stereo initialize is not allowed on a monocular map.");

      ValidateTracker(trackerId);

      for (MapPoint * pMP : mapPoints)
      {
         mMap.AddMapPoint(pMP);
      }

      // Insert KeyFrame in the map
      mMap.mvpKeyFrameOrigins.push_back(pKF);
      mMap.AddKeyFrame(pKF);

      if (mLocalMapper.InsertKeyFrame(pKF))
      {
         UpdateTrackerStatus(trackerId, mapPoints);
         UpdateTrackerStatus(trackerId, pKF);
         mInitialized = true;
      }
      else
      {
         mMap.Clear();
         throw exception("Unable to InsertKeyFrame during InitializeStereo.");
      }
      Print("end InitializeStereo");
   }

   bool MapperServer::InsertKeyFrame(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF)
   {
      ValidateTracker(trackerId);

      if (mLocalMapper.InsertKeyFrame(pKF))
      {
         for (auto pMP : mapPoints)
         {
            mMap.AddMapPoint(pMP);
         }

         // In the original code, this was not done until LocalMapping::ProcessNewKeyFrame
         mMap.AddKeyFrame(pKF);

         // stereo and RGBD modes will create MapPoints
         UpdateTrackerStatus(trackerId, mapPoints);
         UpdateTrackerStatus(trackerId, pKF);

         return true;
      }
      else
         return false;
   }

   void MapperServer::UpdateTrackerStatus(unsigned int trackerId, KeyFrame * pKF)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      // update TrackerStatus array with next KeyFrameId and MapPointId
      if (pKF)
      {
         assert((pKF->GetId() - trackerId) % KEYFRAME_ID_SPAN == 0);
         if (mTrackers[trackerId].nextKeyFrameId <= pKF->GetId())
         {
            mTrackers[trackerId].nextKeyFrameId = pKF->GetId() + KEYFRAME_ID_SPAN;
         }
      }
   }

   void MapperServer::UpdateTrackerStatus(unsigned int trackerId, vector<MapPoint *> mapPoints)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (auto pMP : mapPoints)
      {
         assert((pMP->GetId() - trackerId) % MAPPOINT_ID_SPAN == 0);
         if (mTrackers[trackerId].nextMapPointId <= pMP->GetId())
         {
            mTrackers[trackerId].nextMapPointId = pMP->GetId() + MAPPOINT_ID_SPAN;
         }
      }
   }

   void MapperServer::LoginTracker(
      const cv::Mat & pivotCalib,
      unsigned int & trackerId,
      unsigned long  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      unsigned long & firstMapPointId,
      unsigned int & mapPointIdSpan)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

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

      trackerId = id;
      firstKeyFrameId = mTrackers[id].nextKeyFrameId;
      keyFrameIdSpan = KEYFRAME_ID_SPAN;
      firstMapPointId = mTrackers[id].nextMapPointId;
      mapPointIdSpan = MAPPOINT_ID_SPAN;
   }

   void MapperServer::LogoutTracker(unsigned int id)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      mTrackers[id].connected = false;
   }

   void MapperServer::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      ValidateTracker(trackerId);

      mTrackers[trackerId].poseTcw = poseTcw.clone();
   }

   vector<cv::Mat> MapperServer::GetTrackerPoses()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (int i = 0; i < MAX_TRACKERS; i++)
      {
         if (mTrackers[i].connected)
            poses.push_back(mTrackers[i].poseTcw.clone());
      }
      return poses;
   }

   vector<cv::Mat> MapperServer::GetTrackerPivots()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (int i = 0; i < MAX_TRACKERS; i++)
      {
         if (mTrackers[i].connected)
            poses.push_back(mTrackers[i].pivotCalib.clone());
      }
      return poses;
   }

   Map & MapperServer::GetMap()
   {
      return mMap;
   }

   std::mutex & MapperServer::GetMutexMapUpdate()
   {
      return mMutexMapUpdate;
   }

   void MapperServer::ResetTrackerStatus()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (int i = 0; i < MAX_TRACKERS; ++i)
      {
         mTrackers[i].connected = false;
         mTrackers[i].nextKeyFrameId = i;
         mTrackers[i].nextMapPointId = i;
         mTrackers[i].pivotCalib = cv::Mat::eye(4, 4, CV_32F);
         mTrackers[i].poseTcw = cv::Mat::eye(4, 4, CV_32F);
      }
   }

   void MapperServer::ValidateTracker(unsigned int trackerId)
   {
      if (!mTrackers[trackerId].connected)
         throw exception(string("Tracker is not logged in! Id=").append(to_string(trackerId)).c_str());
   }

}