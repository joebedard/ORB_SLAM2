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
   class Mapper : SyncPrint
   {
   public:

      Mapper(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular);

      virtual long unsigned  KeyFramesInMap();

      virtual void Reset();

      virtual std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

      virtual bool GetPauseRequested();

      virtual bool AcceptKeyFrames();

      virtual void Shutdown();

      bool InsertKeyFrame(unsigned int trackerId, KeyFrame* pKF);

      virtual void Initialize(unsigned int trackerId);

      virtual bool GetInitialized();

      unsigned int LoginTracker(unsigned long  & firstKeyFrameId, unsigned int & keyFrameIdSpan, unsigned long & firstMapPointId, unsigned int & mapPointIdSpan);

      void LogoutTracker(unsigned int id);

      Map * GetMap();

      class Observer
      {
      public:
          virtual void HandleReset() {}
      };

      void AddObserver(Observer * ob);

      void RemoveObserver(Observer * ob);

   private:
      static const unsigned int MAX_TRACKERS = 2;

      static const unsigned int KEYFRAME_ID_SPAN = MAX_TRACKERS;

      /*
      The Local Mapper does not create KeyFrames, but it does create MapPoints. This is why the 
      MAPPOINT_ID_SPAN is one more than the KEYFRAME_ID_SPAN. This set of MapPoint Ids is reserved
      for the Local Mapper.
      */
      static const unsigned int MAPPOINT_ID_SPAN = MAX_TRACKERS + 1;

      static const unsigned long FIRST_MAPPOINT_ID_LOCALMAPPER = MAX_TRACKERS;

      struct TrackerStatus {
         bool connected;
         unsigned long nextKeyFrameId;
         unsigned long nextMapPointId;
      };

      TrackerStatus mTrackers[MAX_TRACKERS];

      std::mutex mMutexLogin;

      ORBVocabulary * mpVocab;

      bool mbMonocular;

      KeyFrameDatabase * mpKeyFrameDB;

      Map * mpMap;

      bool mInitialized;

      LocalMapping * mpLocalMapper;

      LoopClosing * mpLoopCloser;

      std::thread * mptLocalMapping;

      std::thread * mptLoopClosing;

      std::map<Observer *, Observer *> mObservers;

      void NotifyReset();

      void ResetTrackerStatus();

   };

}

#endif // MAPPER_H