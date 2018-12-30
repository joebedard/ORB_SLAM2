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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <opencv2/core/core.hpp>
#include <mutex>
#include <atomic>
#include <unordered_map>

#include "Typedefs.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

namespace ORB_SLAM2
{
   class KeyFrame;
   class Map;
   class Frame;

   class MapPoint : protected SyncPrint
   {
      friend Map;

   public:

      // de-serialization constructor
      MapPoint(id_type id);

      // tracking or mapping constructor
      MapPoint(id_type id, const cv::Mat &Pos, KeyFrame* pRefKF);

      void SetWorldPos(const cv::Mat &Pos);
      cv::Mat GetWorldPos();

      cv::Mat GetNormal();
      KeyFrame* GetReferenceKeyFrame();

      map<KeyFrame*, size_t> GetObservations();
      size_t Observations();

      int GetIndexInKeyFrame(KeyFrame* pKF);
      bool IsObserving(KeyFrame* pKF);

      bool IsBad();

      MapPoint * GetReplaced();
      static MapPoint * FindFinalReplacement(MapPoint * pMP);

      void IncreaseVisible(int n = 1);
      void IncreaseFound(int n = 1);

      // total frames where this point was an inlier / total frames where this MapPoint was visible (inlier or outlier)
      float GetFoundRatio();

      inline int GetFound() {
         return mnFound;
      }

      void ComputeDistinctiveDescriptors();

      cv::Mat GetDescriptor();

      void UpdateNormalAndDepth();

      float GetMinDistanceInvariance();
      float GetMaxDistanceInvariance();
      int PredictScale(const float &currentDist, KeyFrame*pKF);
      int PredictScale(const float &currentDist, Frame* pF);

      bool GetModified();
      void SetModified(bool b);

      static MapPoint * Find(const id_type id, const Map & rMap, unordered_map<id_type, MapPoint *> & newMapPoints);

      static size_t GetVectorBufferSize(const vector<MapPoint *> & mpv);

      static void * ReadVector(
         void * buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         unordered_map<id_type, MapPoint *> & newMapPoints,
         vector<MapPoint *> & mpv);

      static void * WriteVector(
         void * buffer,
         vector<MapPoint *> & mpv);

      static size_t GetSetBufferSize(const set<MapPoint *> & mps);

      static void * ReadSet(
         void * buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames, 
         unordered_map<id_type, MapPoint *> & newMapPoints, 
         set<MapPoint *> & mps);

      static void * WriteSet(
         void * buffer,
         set<MapPoint *> & mps);

      size_t GetBufferSize();

      void * ReadBytes(
         void * const buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames, 
         unordered_map<id_type, MapPoint *> & newMapPoints);

      void * WriteBytes(void * const buffer);

   public:
      
      const id_type & id;

      const id_type & firstKFid;

      // Variables used by the tracking
      float mTrackProjX;
      float mTrackProjY;
      float mTrackProjXR;
      bool mbTrackInView;
      int mnTrackScaleLevel;
      float mTrackViewCos;
      unsigned long int mnTrackReferenceForFrame;
      unsigned long int mnLastFrameSeen;

      // Variables used by local mapping
      unsigned long int mnBALocalForKF;
      unsigned long int mnFuseCandidateForKF;

      // Variables used by loop closing
      unsigned long int mnLoopPointForKF;
      unsigned long int mnCorrectedByKF;
      unsigned long int mnCorrectedReference;
      cv::Mat mPosGBA;
      unsigned long int mnBAGlobalForKF;

      static mutex mGlobalMutex;

   protected:

      // read-only access to mnObs
      const atomic_size_t & nObs;

      mutex mMutexFeatures;

      // Keyframes observing the point and associated index in keyframe
      map<KeyFrame*, size_t> mObservations;

      // called from Map::Link when all linking is complete
      // pre: the thread has locked this->mMutexFeatures and rKF.mMutexFeatures
      void MapPoint::CompleteLink(size_t idx, KeyFrame & rKF);

      // called from Map::Unlink (and Map::Link) when all unlinking is complete
      // pre: the thread has locked this->mMutexFeatures and rKF.mMutexFeatures
      void MapPoint::CompleteUnlink(size_t idx, KeyFrame & rKF);

   private:

      id_type mnFirstKFid;

      atomic_size_t mnObs;

      // Position in absolute coordinates
      cv::Mat mWorldPos;

      // Mean viewing direction
      cv::Mat mNormalVector;

      // Best descriptor to fast matching
      cv::Mat mDescriptor;

      // Reference KeyFrame
      KeyFrame * mpRefKF;

      // Tracking counters
      int mnVisible;
      int mnFound;

      // Bad flag (we do not currently erase MapPoint from memory)
      atomic_bool mbBad;

      // the MapPoint that replaced this MapPoint
      MapPoint * mpReplaced;

      // Scale invariance distances
      float mfMinDistance;
      float mfMaxDistance;

      mutex mMutexPos;

   private:
      id_type mnId;

      atomic_bool mModified;

      struct Header
      {
         id_type mnId;
         id_type mnFirstKFId;
         int mnObs;
         id_type mpRefKFId;
         int mnVisible;
         int mnFound;
         bool mbBad;
         id_type mpReplacedId;
         float mfMinDistance;
         float mfMaxDistance;
      };

      struct Observation
      {
         id_type keyFrameId;
         size_t index;
      };

      static id_type PeekId(void * const buffer);

      static void * ReadObservations(
         void * const buffer,
         const Map & rMap,
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         map<KeyFrame *, size_t> & observations);

      static void * WriteObservations(void * const buffer, map<KeyFrame *, size_t> & observations);

   };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
