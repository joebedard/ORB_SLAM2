/**
* This file is part of ORB-SLAM2-NET.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
* For more information see <https://github.com/joebedard/ORB_SLAM2_NET>
*
* ORB-SLAM2-NET is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2-NET is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2-NET. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

   class KeyFrame;
   class Map;
   class Frame;

   class MapPoint
   {
   public:
      // constructor for map points
      MapPoint(unsigned long int id, const cv::Mat &Pos, KeyFrame* pRefKF);

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

   public:
      unsigned long int MapPoint::GetId();

      long int mnFirstKFid;
      int nObs;

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
      unsigned long int mnId;
   };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
