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

namespace ORB_SLAM2_TEAM
{

   MapperServer::MapperServer(ORBVocabulary & vocab, const bool bMonocular, const unsigned int maxTrackers) :
      SyncPrint("MapperServer: ")
      , mVocab(vocab)
      , mbMonocular(bMonocular)
      , mKeyFrameDB(vocab)
      , mMaxTrackers(maxTrackers)
      , mKeyFrameIdSpan(maxTrackers)

      // The Local Mapper does not create KeyFrames, but it does create MapPoints. Thus, the MAPPOINT_ID_SPAN
      // is one more than the KEYFRAME_ID_SPAN. This set of MapPoint Ids is reserved for the Local Mapper.
      , mMapPointIdSpan(maxTrackers + 1)
      , mFirstMapPointIdMapper(maxTrackers)
      , mTrackerStatus(maxTrackers)
      , mPivotCalib(maxTrackers)
      , mPoseTcw(maxTrackers)
      , mInitialized(false)
      , mLocalMapper(mMap, mKeyFrameDB, mVocab, bMonocular, maxTrackers, mKeyFrameIdSpan, mFirstMapPointIdMapper, mMapPointIdSpan)
      , mLoopCloser(mMap, mKeyFrameDB, mVocab, !bMonocular)
      , mLocalMappingObserver(this)
      , mLoopClosingObserver(this)
   {
      ResetTrackerStatus();

      //Initialize and start the Local Mapping thread
      mLocalMapper.AddObserver(&mLocalMappingObserver);
      mptLocalMapping = new thread(&ORB_SLAM2_TEAM::LocalMapping::Run, &mLocalMapper);

      //Initialize and start the Loop Closing thread
      mLoopCloser.AddObserver(&mLoopClosingObserver);
      mptLoopClosing = new thread(&ORB_SLAM2_TEAM::LoopClosing::Run, &mLoopCloser);

      mLocalMapper.SetLoopCloser(&mLoopCloser);
      mLoopCloser.SetLocalMapper(&mLocalMapper);
   }

   MapperServer::~MapperServer()
   {
      Print("begin ~MapperServer");
      mLocalMapper.RequestFinish();
      mLoopCloser.RequestFinish();

      mptLocalMapping->join();
      mptLoopClosing->join();

      delete mptLocalMapping;
      delete mptLoopClosing;
      Print("end ~MapperServer");
   }

   unsigned long MapperServer::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   unsigned long MapperServer::MapPointsInMap()
   {
      return mMap.MapPointsInMap();
   }

   unsigned int MapperServer::LoopsInMap()
   {
      return mLoopCloser.quantityLoops;
   }

   void MapperServer::Reset()
   {
      // Reset Local Mapping
      Print("Begin Local Mapper Reset");
      mLocalMapper.RequestReset();
      Print("End Local Mapper Reset");

      // Reset Loop Closing
      Print("Begin Loop Closing Reset");
      mLoopCloser.RequestReset();
      Print("End Loop Closing Reset");

      Print("waiting to lock map");
      unique_lock<mutex> lock(mMap.mutexMapUpdate);
      Print("map is locked");

      ResetTrackerStatus();

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

   bool MapperServer::GetIdle()
   {
      return mLocalMapper.GetIdle();
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

      if (mLocalMapper.InitializeMono(trackerId, pKF1, pKF2, mapPoints))
      {
         UpdateTrackerStatus(trackerId, mapPoints);
         UpdateTrackerStatus(trackerId, pKF1);
         UpdateTrackerStatus(trackerId, pKF2);
         mInitialized = true;
      }
      else
      {
         mMap.Clear();
         throw exception("Mapper failed to initialize monocular map.");
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

      if (mLocalMapper.InitializeStereo(trackerId, pKF, mapPoints))
      {
         UpdateTrackerStatus(trackerId, mapPoints);
         UpdateTrackerStatus(trackerId, pKF);
         mInitialized = true;
      }
      else
      {
         mMap.Clear();
         throw exception("Mapper failed to initialize stereo map.");
      }
      Print("end InitializeStereo");
   }

   bool MapperServer::InsertKeyFrame(unsigned int trackerId, KeyFrame * pKF, vector<MapPoint *> & createdMapPoints, vector<MapPoint *> & updatedMapPoints)
   {
      Print("begin InsertKeyFrame");
      ValidateTracker(trackerId);

      if (mLocalMapper.InsertKeyFrame(trackerId, pKF, createdMapPoints, updatedMapPoints))
      {
         // stereo and RGBD modes will create MapPoints
         UpdateTrackerStatus(trackerId, createdMapPoints);
         UpdateTrackerStatus(trackerId, pKF);

         Print("end InsertKeyFrame 1");
         return true;
      }
      else
      {
         Print("end InsertKeyFrame 2");
         return false;
      }
   }

   void MapperServer::UpdateTrackerStatus(unsigned int trackerId, KeyFrame * pKF)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      // update TrackerStatus array with next KeyFrameId and MapPointId
      if (pKF)
      {
         assert((pKF->id - trackerId) % mKeyFrameIdSpan == 0);
         if (mTrackerStatus[trackerId].nextKeyFrameId <= pKF->id)
         {
            mTrackerStatus[trackerId].nextKeyFrameId = pKF->id + mKeyFrameIdSpan;
         }
      }
   }

   void MapperServer::UpdateTrackerStatus(unsigned int trackerId, vector<MapPoint *> mapPoints)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (MapPoint * pMP : mapPoints)
      {
         if (pMP)
         {
            assert((pMP->id - trackerId) % mMapPointIdSpan == 0);
            if (mTrackerStatus[trackerId].nextMapPointId <= pMP->id)
               mTrackerStatus[trackerId].nextMapPointId = pMP->id + mMapPointIdSpan;
         }
      }
   }

   void MapperServer::LoginTracker(
      const cv::Mat & pivotCalib,
      unsigned int & trackerId,
      id_type  & firstKeyFrameId,
      unsigned int & keyFrameIdSpan,
      id_type & firstMapPointId,
      unsigned int & mapPointIdSpan)
   {
      Print("begin LoginTracker");
      unique_lock<mutex> lock(mMutexTrackerStatus);

      unsigned int id;
      for (id = 0; id < mMaxTrackers; ++id)
      {
         if (!mTrackerStatus[id].connected)
         {
            mTrackerStatus[id].connected = true;
            mPivotCalib[id] = pivotCalib;
            break;
         }
      }

      if (id >= mMaxTrackers)
         throw std::exception("Maximum number of trackers reached. Additional trackers are not supported.");

      trackerId = id;
      firstKeyFrameId = mTrackerStatus[id].nextKeyFrameId;
      keyFrameIdSpan = mKeyFrameIdSpan;
      firstMapPointId = mTrackerStatus[id].nextMapPointId;
      mapPointIdSpan = mMapPointIdSpan;
      Print("end LoginTracker");
   }

   void MapperServer::LogoutTracker(unsigned int id)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      mTrackerStatus[id].connected = false;
   }

   void MapperServer::UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw)
   {
      Print("begin UpdatePose");
      unique_lock<mutex> lock(mMutexTrackerStatus);

      ValidateTracker(trackerId);

      mPoseTcw[trackerId] = poseTcw.clone();
      Print("end UpdatePose");
   }

   vector<cv::Mat> MapperServer::GetTrackerPoses()
   {
      //Print("begin GetTrackerPoses");
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (unsigned int i = 0; i < mMaxTrackers; i++)
      {
         poses.push_back(mPoseTcw[i].clone());
      }

      //Print("end GetTrackerPoses");
      return poses;
   }

   vector<cv::Mat> MapperServer::GetTrackerPivots()
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      vector<cv::Mat> poses;
      for (unsigned int i = 0; i < mMaxTrackers; i++)
      {
         poses.push_back(mPivotCalib[i].clone());
      }
      return poses;
   }

   Map & MapperServer::GetMap()
   {
      return mMap;
   }

   std::mutex & MapperServer::GetMutexMapUpdate()
   {
      return mMap.mutexMapUpdate;
   }

   void MapperServer::ResetTrackerStatus()
   {
      Print("begin ResetTrackerStatus");
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (unsigned int i = 0; i < mMaxTrackers; ++i)
      {
         mTrackerStatus[i].connected = false;
         mTrackerStatus[i].nextKeyFrameId = i;
         mTrackerStatus[i].nextMapPointId = i;
         mPivotCalib[i] = cv::Mat::eye(4, 4, CV_32F);
         mPoseTcw[i] = cv::Mat::eye(4, 4, CV_32F);
      }
      Print("end ResetTrackerStatus");
   }

   void MapperServer::ValidateTracker(unsigned int trackerId)
   {
      if (!mTrackerStatus[trackerId].connected)
         throw exception(string("Tracker is not logged in! Id=").append(to_string(trackerId)).c_str());
   }

   forward_list<Statistics> MapperServer::GetStatistics()
   {
      mLocalMapper.RequestFinish();
      mLoopCloser.RequestFinish();
      mptLocalMapping->join();
      mptLoopClosing->join();
      forward_list<Statistics> stats(mLoopCloser.GetStatistics());
      stats.push_front(mLocalMapper.GetStatistics());
      return stats;
   }
}