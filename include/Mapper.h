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

#ifndef MAPPER_H
#define MAPPER_H

#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Enums.h"

namespace ORB_SLAM2
{

   class LocalMapping;
   class LoopClosing;

   // interface for all Mapping functionality
   class Mapper
   {
   public:

      Mapper(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular);

      virtual std::mutex & getMutexMapUpdate();

      virtual long unsigned  KeyFramesInMap();

      virtual void EraseKeyFrame(KeyFrame* pKF);

      virtual void AddKeyFrame(KeyFrame *pKF);

      virtual long unsigned int NextMapPointId();

      virtual void EraseMapPoint(MapPoint* pMP);

      virtual void AddMapPoint(MapPoint* pMP);

      virtual long unsigned int MapPointsInMap();

      virtual std::vector<MapPoint*> GetAllMapPoints();

      virtual void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

      virtual void AddOriginKeyFrame(KeyFrame* pKF);

      virtual void Reset();

      virtual std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

      virtual void SetState(eTrackingState state);

      virtual eTrackingState GetState();

      virtual void RequestStop();

      virtual bool isStopped();

      virtual void Release();

      virtual bool stopRequested();

      virtual bool AcceptKeyFrames();

      virtual void InterruptBA();

      virtual int KeyframesInQueue();

      virtual bool SetNotStop(bool b);

      virtual void InsertKeyFrame(KeyFrame* pKF);

      virtual void GlobalBundleAdjustemnt(int nIterations = 5, bool *pbStopFlag = NULL,
         const unsigned long nLoopKF = 0, const bool bRobust = true);

      virtual void Shutdown();

      virtual KeyFrame * CreateNewKeyFrame(Frame & currentFrame, int sensorType);

      virtual void Initialize(Map & pMap);

   private:
      ORBVocabulary * mpVocab;
      bool mbMonocular;
      KeyFrameDatabase * mpKeyFrameDB;
      Map* mpMap;
      static long unsigned int nNextMapPointId;
      eTrackingState mState;

      // initialization variables
      bool mInitialized;
      LocalMapping* mpLocalMapper;
      LoopClosing* mpLoopCloser;

      // System threads: Local Mapping, Loop Closing
      std::thread* mptLocalMapping;
      std::thread* mptLoopClosing;
   };

}

#endif // MAPPER_H