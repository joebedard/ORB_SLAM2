/**
* This file is part of ORB-SLAM2-TEAM.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "MapObserver.h"
#include "MapSubject.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "MapperSubject.h"
#include "SyncPrint.h"

#include <mutex>

namespace ORB_SLAM2
{
   class Tracking;
   class LoopClosing;
   class Map;
   class MapperServer;

   class LocalMapping : public MapperSubject, protected SyncPrint
   {
   public:
      LocalMapping(
         Map & map,
         std::mutex & mutexMapUpdate, 
         KeyFrameDatabase & keyFrameDB,
         ORBVocabulary & vocab,
         const float bMonocular,
         unsigned long firstMapPointId,
         unsigned int mapPointIdSpan
      );

      void SetLoopCloser(LoopClosing * pLoopCloser);

      // Main function
      void Run();

      bool InsertKeyFrame(KeyFrame * pKF, vector<MapPoint *> & mapPoints);

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
      bool IsFinished();

      int KeyframesInQueue()
      {
         unique_lock<mutex> lock(mMutexNewKFs);
         return mlNewKeyFrames.size();
      }

   protected:

      bool CheckNewKeyFrames();

      void ProcessNewKeyFrame(MapChangeEvent & mapChanges);

      void CreateNewMapPoints(MapChangeEvent & mapChanges);

      // iterate through the new MapPoints (that were added by the new KeyFrame) and remove low-quality MapPoints
      void MapPointCulling(MapChangeEvent & mapChanges);

      void SearchInNeighbors(MapChangeEvent & mapChanges);

      void KeyFrameCulling(MapChangeEvent & mapChanges);

      cv::Mat ComputeF12(KeyFrame * &pKF1, KeyFrame * &pKF2);

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

      Map & mMap;

      KeyFrameDatabase & mKeyFrameDB;

      ORBVocabulary & mVocab;

      LoopClosing * mpLoopCloser;

      list<KeyFrame *> mlNewKeyFrames;

      KeyFrame * mpCurrentKeyFrame;

      list<MapPoint *> mlpRecentAddedMapPoints;

      mutex mMutexNewKFs;

      bool mbAbortBA;

   private:

      mutex & mMutexMapUpdate;

      unsigned long mNextMapPointId;

      unsigned int mMapPointIdSpan;

      bool mbPaused;

      bool mbPauseRequested;

      bool mbNotPause;

      mutex mMutexPause;

      bool mbAcceptKeyFrames;

      mutex mMutexAccept;

      unsigned long NewMapPointId();

   };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
