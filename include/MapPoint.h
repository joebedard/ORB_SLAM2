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

#include "Typedefs.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

namespace ORB_SLAM2
{
   class KeyFrame;
   class Map;
   class Frame;

   class MapPoint
   {
   public:

      // constructor for map points
      MapPoint();

      // constructor for map points
      MapPoint(id_type id, const cv::Mat &Pos, KeyFrame* pRefKF);

      id_type MapPoint::GetId();

      void SetWorldPos(const cv::Mat &Pos);
      cv::Mat GetWorldPos();

      cv::Mat GetNormal();
      KeyFrame* GetReferenceKeyFrame();

      std::map<KeyFrame*, size_t> GetObservations();
      int Observations();

      void AddObservation(KeyFrame* pKF, size_t idx);
      void EraseObservation(KeyFrame* pKF, Map * pMap);

      int GetIndexInKeyFrame(KeyFrame* pKF);
      bool IsInKeyFrame(KeyFrame* pKF);

      void SetBadFlag(Map * pMap);
      bool isBad();

      void Replace(MapPoint* pMP, Map * pMap);
      MapPoint* GetReplaced();

      void IncreaseVisible(int n = 1);
      void IncreaseFound(int n = 1);
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

      size_t GetBufferSize();
      void * ReadBytes(const void * data, Map & map, KeyFrame * pNewKF1, KeyFrame * pNewKF2);
      void * WriteBytes(const void * data);

   public:
      id_type mnFirstKFid;

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

      static std::mutex mGlobalMutex;

   private:
      int nObs;

      // Position in absolute coordinates
      cv::Mat mWorldPos;

      // Keyframes observing the point and associated index in keyframe
      std::map<KeyFrame*, size_t> mObservations;

      // Mean viewing direction
      cv::Mat mNormalVector;

      // Best descriptor to fast matching
      cv::Mat mDescriptor;

      // Reference KeyFrame
      KeyFrame* mpRefKF;

      // Tracking counters
      int mnVisible;
      int mnFound;

      // Bad flag (we do not currently erase MapPoint from memory)
      bool mbBad;
      MapPoint* mpReplaced;

      // Scale invariance distances
      float mfMinDistance;
      float mfMaxDistance;

      std::mutex mMutexPos;
      std::mutex mMutexFeatures;

   private:
      id_type mnId;

      struct Header
      {
         id_type mnId;
         id_type mnFirstKFId;
         int nObs;
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

      static void * ReadObservations(const void * buffer, Map & map, KeyFrame * pNewKF1, KeyFrame * pNewKF2, std::map<KeyFrame *, size_t> & observations);

      static void * WriteObservations(const void * buffer, std::map<KeyFrame *, size_t> & observations);

   };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
