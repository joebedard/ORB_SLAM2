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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "SyncPrint.h"
#include "FrameCalibration.h"

#include <mutex>


namespace ORB_SLAM2
{

   class Map;
   class MapPoint;
   class Frame;
   class KeyFrameDatabase;

   class KeyFrame : protected SyncPrint
   {
   public:

      KeyFrame(id_type id, Frame &F);

      id_type GetId();

      // Pose functions
      void SetPose(const cv::Mat &Tcw);
      cv::Mat GetPose();
      cv::Mat GetPoseInverse();
      cv::Mat GetCameraCenter();
      cv::Mat GetRotation();
      cv::Mat GetTranslation();

      // Bag of Words Representation
      void ComputeBoW(ORBVocabulary & vocab);

      // Covisibility graph functions
      void AddConnection(KeyFrame* pKF, const int &weight);
      void EraseConnection(KeyFrame* pKF);
      void UpdateConnections();
      void UpdateBestCovisibles();
      std::set<KeyFrame *> GetConnectedKeyFrames();
      std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
      std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
      std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
      int GetWeight(KeyFrame* pKF);

      // Spanning tree functions
      void AddChild(KeyFrame* pKF);
      void EraseChild(KeyFrame* pKF);
      void ChangeParent(KeyFrame* pKF);
      std::set<KeyFrame*> GetChilds();
      KeyFrame* GetParent();
      bool hasChild(KeyFrame* pKF);

      // Loop Edges
      void AddLoopEdge(KeyFrame* pKF);
      std::set<KeyFrame*> GetLoopEdges();

      // MapPoint observation functions
      void AddMapPoint(MapPoint* pMP, const size_t &idx);
      void EraseMapPointMatch(const size_t &idx);
      void EraseMapPointMatch(MapPoint* pMP);
      void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
      std::set<MapPoint*> GetMapPoints();
      std::vector<MapPoint*> GetMapPointMatches();
      int TrackedMapPoints(const int &minObs);
      MapPoint* GetMapPoint(const size_t &idx);

      // KeyPoint functions
      std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
      cv::Mat UnprojectStereo(int i);

      // Image
      bool IsInImage(const float &x, const float &y) const;

      // Enable/Disable bad flag changes
      void SetNotErase();

      // called by LoopClosing, allows keyframes to be deleted, performs pending deletes
      // returns true if this object is deleted, false otherwise
      bool SetErase(Map* pMap, KeyFrameDatabase* pKeyFrameDB);

      // Set/check bad flag
      // returns true if this object is deleted, false otherwise
      bool SetBadFlag(Map* pMap, KeyFrameDatabase* pKeyFrameDB);

      bool isBad();

      // Compute Scene Depth (q=2 median). Used in monocular.
      float ComputeSceneMedianDepth(const int q);

      size_t GetBufferSize();
      void * ReadBytes(const void * data, Map & map);
      void * WriteBytes(const void * data);

      static bool weightComp(int a, int b) {
         return a > b;
      }

      static bool lId(KeyFrame* pKF1, KeyFrame* pKF2) {
         return pKF1->mnId < pKF2->mnId;
      }


   // The following variables are accesed from only 1 thread or never change (no mutex needed).
   public:

      double & timestamp;

      // Variables used by the tracking
      long unsigned int mnTrackReferenceForFrame;
      long unsigned int mnFuseTargetForKF;

      // Variables used by the local mapping
      long unsigned int mnBALocalForKF;
      long unsigned int mnBAFixedForKF;

      // Variables used by the keyframe database
      long unsigned int mnLoopQuery;
      int mnLoopWords;
      float mLoopScore;
      long unsigned int mnRelocQuery;
      int mnRelocWords;
      float mRelocScore;

      // Variables used by loop closing
      cv::Mat mTcwGBA;
      cv::Mat mTcwBefGBA;
      id_type mnBAGlobalForKF;

      // Calibration parameters
      FrameCalibration mFC;

      // Number of KeyPoints (features).
      const int N;

      // Vector of KeyPoints (features) based on original image(s). Used for visualization.
      const std::vector<cv::KeyPoint> mvKeys;

      // Vector of undistorted KeyPoints (features). Used by tracking and mapping.
      // If it is a stereo frame, mvKeysUn is redundant because images are pre-rectified.
      // If it is a RGB-D frame, the RGB images might be distorted.
      const std::vector<cv::KeyPoint> mvKeysUn;

      // Corresponding stereo coordinate for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      const std::vector<float> mvuRight;

      // Corresponding depth for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      const std::vector<float> mvDepth;

      // Corresponding descriptor for each KeyPoint.
      const cv::Mat mDescriptors;

      //BoW
      DBoW2::BowVector mBowVec;
      DBoW2::FeatureVector mFeatVec;

      // Pose relative to parent (this is computed when bad flag is activated)
      cv::Mat mTcp;

      // Scale
      const int mnScaleLevels;
      const float mfScaleFactor;
      const float mfLogScaleFactor;
      const std::vector<float> mvScaleFactors;
      const std::vector<float> mvLevelSigma2;
      const std::vector<float> mvInvLevelSigma2;


   // The following variables need to be accessed trough a mutex to be thread safe.
   protected:

      // SE3 Pose and camera center
      cv::Mat Tcw;
      cv::Mat Twc;
      cv::Mat Ow;

      // MapPoints associated to KeyPoints (via the index), NULL pointer if no association.
      // Each non-null element corresponds to an element in mvKeysUn.
      std::vector<MapPoint*> mvpMapPoints;

      // Grid over the image to speed up feature matching
      // not serialized, rebuilt with AssignFeaturesToGrid()
      std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

      std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
      std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
      std::vector<int> mvOrderedWeights;

      // Spanning Tree and Loop Edges
      bool mbFirstConnection;
      KeyFrame* mpParent;
      std::set<KeyFrame*> mspChildrens;
      std::set<KeyFrame*> mspLoopEdges;

      // Bad flags
      bool mbNotErase; //server-only
      bool mbToBeErased; //server-only
      bool mbBad;

      std::mutex mMutexPose;
      std::mutex mMutexConnections;
      std::mutex mMutexFeatures;

   private:
      struct Header
      {
         id_type mnId;
         double mTimestamp;
         int N;
         unsigned int quantityKeys;
         unsigned int quantityKeysUn;
         unsigned int quantityKeysRight;
         unsigned int quantityKeysDepth;
         int mnScaleLevels;
         float mfScaleFactor;
         float mfLogScaleFactor;
         unsigned int quantityScaleFactor;
         unsigned int quantityMapPoints;
         unsigned int quantityConnectedKeyFrameWeights;
         unsigned int quantityOrderedConnectedKeyFrames;
         unsigned int quantityOrderedWeights;
         bool mbFirstConnection;
         id_type parentKeyFrameId;
         unsigned int quantityChildren;
         unsigned int quantityLoopEdges;
         bool mbBad;
      };

      id_type mnId;

      double mTimestamp;

      // Grid (to speed up feature matching)
      const int mnGridCols;
      const int mnGridRows;

      bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
      void AssignFeaturesToGrid();
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
