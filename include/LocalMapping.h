/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "MapObserver.h"
#include "MapSubject.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "SyncPrint.h"

#include <mutex>

namespace ORB_SLAM2
{

   class Tracking;
   class LoopClosing;
   class Map;
   class MapperServer;

   class LocalMapping : public MapSubject, protected SyncPrint
   {
   public:
      LocalMapping(
         Map* pMap,
         std::mutex & mutexMapUpdate,
         KeyFrameDatabase* pDB,
         const float bMonocular,
         unsigned long firstMapPointId,
         unsigned int mapPointIdSpan
      );

      void SetLoopCloser(LoopClosing* pLoopCloser);

      // Main function
      void Run();

      bool InsertKeyFrame(vector<MapPoint *> mapPoints, KeyFrame* pKF);

      // Thread Synch
      void RequestPause();
      void RequestReset();
      bool Pause();
      void Resume();
      bool IsPaused();
      bool PauseRequested();
      bool AcceptKeyFrames();
      void SetAcceptKeyFrames(bool flag);
      bool SetNotPause(bool flag);

      void InterruptBA();

      void RequestFinish();
      bool isFinished();

      int KeyframesInQueue()
      {
         unique_lock<mutex> lock(mMutexNewKFs);
         return mlNewKeyFrames.size();
      }

   protected:

      bool CheckNewKeyFrames();

      void ProcessNewKeyFrame(MapChangeEvent & mapChanges);

      void CreateNewMapPoints(MapChangeEvent & mapChanges);

      void MapPointCulling(MapChangeEvent & mapChanges);

      void SearchInNeighbors(MapChangeEvent & mapChanges);

      void KeyFrameCulling(MapChangeEvent & mapChanges);

      cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

      cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

      bool mbMonocular;

      void ResetIfRequested();
      bool mbResetRequested;
      mutex mMutexReset;

      bool CheckFinish();
      void SetFinish();
      bool mbFinishRequested;
      bool mbFinished;
      mutex mMutexFinish;

      Map* mpMap;
      KeyFrameDatabase* mpKeyFrameDB;

      LoopClosing* mpLoopCloser;

      list<KeyFrame*> mlNewKeyFrames;

      KeyFrame* mpCurrentKeyFrame;

      list<MapPoint*> mlpRecentAddedMapPoints;

      mutex mMutexNewKFs;

      bool mbAbortBA;

      bool mbAcceptKeyFrames;

      mutex mMutexAccept;

   private:

      mutex & mMutexMapUpdate;

      unsigned long mNextMapPointId;

      unsigned int mMapPointIdSpan;

      bool mbPaused;

      bool mbPauseRequested;

      bool mbNotPause;

      mutex mMutexPause;

      unsigned long NewMapPointId();
   };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
