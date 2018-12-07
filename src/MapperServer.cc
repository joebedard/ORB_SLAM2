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
      Print("begin ~MapperServer");
      mLocalMapper.RequestFinish();
      mLoopCloser.RequestFinish();

      mptLocalMapping->join();
      mptLoopClosing->join();

      delete mptLocalMapping;
      delete mptLoopClosing;
      Print("end ~MapperServer");
   }

   long unsigned MapperServer::KeyFramesInMap()
   {
      return mMap.KeyFramesInMap();
   }

   void MapperServer::Reset()
   {
      Print("Begin Reset");

      // Reset Local Mapping
      Print("Begin Local Mapper Reset");
      mLocalMapper.RequestReset();
      Print("End Local Mapper Reset");

      // Reset Loop Closing
      Print("Begin Loop Closing Reset");
      mLoopCloser.RequestReset();
      Print("End Loop Closing Reset");

      Print("unique_lock<mutex> lock(mMutexMapUpdate);");
      unique_lock<mutex> lock(mMutexMapUpdate);
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

      vector<MapPoint *> noPoints;
      if (mLocalMapper.InsertKeyFrame(pKF1, noPoints) && mLocalMapper.InsertKeyFrame(pKF2, noPoints))
      {
         pKF1->ComputeBoW(mVocab);
         pKF2->ComputeBoW(mVocab);
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

      // Insert KeyFrame in the map
      mMap.mvpKeyFrameOrigins.push_back(pKF);

      if (mLocalMapper.InsertKeyFrame(pKF, mapPoints))
      {
         pKF->ComputeBoW(mVocab);
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
      Print("begin InsertKeyFrame");
      ValidateTracker(trackerId);

      if (mLocalMapper.InsertKeyFrame(pKF, mapPoints))
      {
         pKF->ComputeBoW(mVocab);

         // stereo and RGBD modes will create MapPoints
         UpdateTrackerStatus(trackerId, mapPoints);
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
         assert((pKF->GetId() - trackerId) % KEYFRAME_ID_SPAN == 0);
         if (mTrackerStatus[trackerId].nextKeyFrameId <= pKF->GetId())
         {
            mTrackerStatus[trackerId].nextKeyFrameId = pKF->GetId() + KEYFRAME_ID_SPAN;
         }
      }
   }

   void MapperServer::UpdateTrackerStatus(unsigned int trackerId, vector<MapPoint *> mapPoints)
   {
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (auto pMP : mapPoints)
      {
         assert((pMP->GetId() - trackerId) % MAPPOINT_ID_SPAN == 0);
         if (mTrackerStatus[trackerId].nextMapPointId <= pMP->GetId())
         {
            mTrackerStatus[trackerId].nextMapPointId = pMP->GetId() + MAPPOINT_ID_SPAN;
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
      for (id = 0; id < MAX_TRACKERS; ++id)
      {
         if (!mTrackerStatus[id].connected)
         {
            mTrackerStatus[id].connected = true;
            mPivotCalib[id] = pivotCalib;
            break;
         }
      }

      if (id >= MAX_TRACKERS)
         throw std::exception("Maximum number of trackers reached. Additional trackers are not supported.");

      trackerId = id;
      firstKeyFrameId = mTrackerStatus[id].nextKeyFrameId;
      keyFrameIdSpan = KEYFRAME_ID_SPAN;
      firstMapPointId = mTrackerStatus[id].nextMapPointId;
      mapPointIdSpan = MAPPOINT_ID_SPAN;
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
      for (int i = 0; i < MAX_TRACKERS; i++)
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
      for (int i = 0; i < MAX_TRACKERS; i++)
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
      return mMutexMapUpdate;
   }

   void MapperServer::ResetTrackerStatus()
   {
      Print("begin ResetTrackerStatus");
      unique_lock<mutex> lock(mMutexTrackerStatus);

      for (int i = 0; i < MAX_TRACKERS; ++i)
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

}