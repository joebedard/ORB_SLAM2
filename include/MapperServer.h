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

#ifndef MAPPERSERVER_H
#define MAPPERSERVER_H

#include "Map.h"
#include "MapChangeEvent.h"
#include "Mapper.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapperObserver.h"
#include "MapperSubject.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Enums.h"

namespace ORB_SLAM2_TEAM
{

   class LocalMapping;
   class LoopClosing;

   // interface for all Mapping functionality
   class MapperServer : public Mapper, protected SyncPrint
   {
   public:

      // Pre: vocab is loaded
      MapperServer(ORBVocabulary & vocab, const bool bMonocular, const unsigned int maxTrackers);

      ~MapperServer();

      virtual unsigned long KeyFramesInMap();

      virtual unsigned long MapPointsInMap();

      virtual unsigned int LoopsInMap();

      virtual void Reset();

      virtual std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame * F);

      virtual bool GetPauseRequested();

      virtual bool GetIdle();

      virtual bool InsertKeyFrame(unsigned int trackerId, KeyFrame * pKF, vector<MapPoint *> & createdMapPoints, vector<MapPoint *> & updatedMapPoints);

      virtual void InitializeMono(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2);

      virtual void InitializeStereo(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF);

      virtual bool GetInitialized();

      virtual Map & GetMap();

      virtual std::mutex & GetMutexMapUpdate();

      virtual void LoginTracker(
         const cv::Mat & pivotCalib,
         unsigned int & trackerId,
         id_type  & firstKeyFrameId,
         unsigned int & keyFrameIdSpan,
         id_type & firstMapPointId,
         unsigned int & mapPointIdSpan);

      virtual void LogoutTracker(unsigned int id);

      virtual void UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw);

      virtual vector<cv::Mat> GetTrackerPoses();

      virtual vector<cv::Mat> GetTrackerPivots();

      virtual void Shutdown();

      // NOTE: Call after tracking ends. Not thread-safe! Stops the mapping and loop closing threads.
      virtual list<Statistics> GetStatistics();

      // NOTE: Call after tracking ends. Not thread-safe! Stops the mapping and loop closing threads.
      virtual void WriteMetrics(ofstream & ofs);

   private:
      const unsigned int mMaxTrackers;

      const unsigned int mKeyFrameIdSpan;

      const unsigned int mMapPointIdSpan;

      const unsigned int mFirstMapPointIdMapper;

      struct TrackerStatus {
         bool connected;
         unsigned long nextKeyFrameId;
         unsigned long nextMapPointId;
      };

      vector<TrackerStatus> mTrackerStatus;

      vector<cv::Mat> mPivotCalib;

      vector<cv::Mat> mPoseTcw;

      std::mutex mMutexTrackerStatus;

      ORBVocabulary & mVocab;

      bool mbMonocular;

      KeyFrameDatabase mKeyFrameDB;

      Map mMap;

      bool mInitialized;

      bool mFinalized;

      LocalMapping mLocalMapper;

      LoopClosing mLoopCloser;

      std::thread * mptLocalMapping;

      std::thread * mptLoopClosing;

      void ResetTrackerStatus();

      void UpdateTrackerStatus(unsigned int trackerId, KeyFrame * pKF);

      void UpdateTrackerStatus(unsigned int trackerId, vector<MapPoint *> mapPoints);

      void ValidateTracker(unsigned int trackerId);

      class PrivateMapperObserver : public MapperObserver
      {
         MapperServer * mpMapperServer;
      public:
         PrivateMapperObserver(MapperServer * pMapperServer) : mpMapperServer(pMapperServer) {}
         virtual void HandleMapReset() { mpMapperServer->NotifyMapReset(); }
         virtual void HandleMapChanged(MapChangeEvent & mce) { mpMapperServer->NotifyMapChanged(mce); }
         virtual void HandlePauseRequested(bool b) { mpMapperServer->NotifyPauseRequested(b); }
         virtual void HandleIdle(bool b) { mpMapperServer->NotifyIdle(b); }
      };

      PrivateMapperObserver mLocalMappingObserver;

      class PrivateMapObserver : public MapObserver
      {
         MapperServer * mpMapperServer;
      public:
         PrivateMapObserver(MapperServer * pMapperServer) : mpMapperServer(pMapperServer) {}
         virtual void HandleMapReset() { mpMapperServer->NotifyMapReset(); }
         virtual void HandleMapChanged(MapChangeEvent & mce) { mpMapperServer->NotifyMapChanged(mce); }
      };

      PrivateMapObserver mLoopClosingObserver;
   };

}

#endif // MAPPERSERVER_H