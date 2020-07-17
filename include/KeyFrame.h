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
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "SyncPrint.h"
#include "FrameCalibration.h"

#include <mutex>
#include <atomic>
#include <unordered_map>


namespace ORB_SLAM2_TEAM
{

   class Map;
   class MapPoint;
   class Frame;
   class KeyFrameDatabase;


   class KeyFrame : protected SyncPrint
   {

      friend Map;

   public:

      KeyFrame(id_type id);

      KeyFrame(id_type id, Frame &F);

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
      void AddConnection(KeyFrame * pKF, const int &weight);
      void EraseConnection(KeyFrame * pKF);
      void UpdateConnections();
      void UpdateBestCovisibles();
      set<KeyFrame *> GetConnectedKeyFrames();
      vector<KeyFrame * > GetVectorCovisibleKeyFrames();
      vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int & n);
      vector<KeyFrame *> GetCovisiblesByWeight(const int & w);
      int GetWeight(KeyFrame * pKF);

      // Spanning tree functions
      void AddChild(KeyFrame * pKF);
      void EraseChild(KeyFrame * pKF);
      void ChangeParent(KeyFrame * pKF);
      set<KeyFrame *> GetChilds();
      KeyFrame * GetParent();
      bool hasChild(KeyFrame * pKF);

      // Loop Edges
      void AddLoopEdge(KeyFrame * pKF);
      set<KeyFrame *> GetLoopEdges();

      // MapPoint observation functions
      set<MapPoint*> GetMapPoints();
      vector<MapPoint*> GetMapPointMatches();
      int TrackedMapPoints(const int &minObs);
      MapPoint* GetMapPoint(const size_t idx);

      // KeyPoint functions
      vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
      cv::Mat UnprojectStereo(int i);

      // Image
      bool IsInImage(const float &x, const float &y) const;

      // Enable/Disable bad flag changes
      void SetNotErase();

      // called by LoopClosing, allows keyframes to be deleted, performs pending deletes
      // returns true if this object is deleted, false otherwise
      bool SetErase(Map* pMap, KeyFrameDatabase* pKeyFrameDB);

      // Set/check bad flag
      // if this object is deleted, MapPoints are unlinked
      // returns true if this object is deleted, false otherwise
      bool SetBadFlag(Map* pMap, KeyFrameDatabase* pKeyFrameDB);

      bool IsBad();

      // Compute Scene Depth (q=2 median). Used in monocular.
      float ComputeSceneMedianDepth(const int q);

      bool GetModified();
      void SetModified(bool b);

      static KeyFrame * Find(id_type id, const Map & rMap, unordered_map<id_type, KeyFrame *> & newKeyFrames);

      static void * Read(
         void * buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         unordered_map<id_type, MapPoint *> & newMapPoints,
         KeyFrame ** const ppKF);

      static size_t GetVectorBufferSize(const vector<KeyFrame *> & kfv);

      static void * ReadVector(
         void * buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         unordered_map<id_type, MapPoint *> & newMapPoints,
         vector<KeyFrame *> & kfv);

      static size_t GetSetBufferSize(const set<KeyFrame *> & kfs);

      static void * ReadSet(
         void * buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         unordered_map<id_type, MapPoint *> & newMapPoints,
         set<KeyFrame *> & kfs);

      static void * WriteSet(
         void * buffer,
         set<KeyFrame *> & kfs);

      size_t GetBufferSize();

      void * ReadBytes(
         const void * data, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames, 
         unordered_map<id_type, MapPoint *> & newMapPoints);

      void * WriteBytes(const void * data);

      static bool weightComp(int a, int b) {
         return a > b;
      }

      static bool lId(KeyFrame * pKF1, KeyFrame * pKF2) {
         return pKF1->mnId < pKF2->mnId;
      }


   // The following variables are accesed from only 1 thread or never change (no mutex needed).
   public:

      const id_type & id;

      const double & timestamp;

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
      const int & N;

      // Vector of undistorted KeyPoints (features). Used by tracking and mapping.
      // If it is a stereo frame, mvKeysUn is redundant because images are pre-rectified.
      // If it is a RGB-D frame, the RGB images might be distorted.
      const vector<cv::KeyPoint> & keysUn;

      // Corresponding stereo coordinate for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      const vector<float> & right;

      // Corresponding depth for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      const vector<float> & depth;

      // Corresponding descriptor for each KeyPoint.
      const cv::Mat & descriptors;

      //BoW
      DBoW2::BowVector mBowVec;
      DBoW2::FeatureVector mFeatVec;

      // Pose relative to parent (this is computed when bad flag is activated)
      const cv::Mat & Tcp;

      // Scale
      const int & scaleLevels;
      const float & scaleFactor;
      const float & logScaleFactor;
      const vector<float> & scaleFactors;
      const vector<float> & levelSigma2;
      const vector<float> & invLevelSigma2;


   // The following variables need to be accessed trough a mutex to be thread safe.
   protected:

      // SE3 Pose and camera center
      cv::Mat Tcw;
      cv::Mat Twc;
      cv::Mat Ow;

      // MapPoints associated to KeyPoints (via the index), NULL pointer if no association.
      // Each non-null element corresponds to an element in mvKeysUn.
      vector<MapPoint*> mvpMapPoints;

      // Grid over the image to speed up feature matching
      // not serialized, rebuilt with AssignFeaturesToGrid()
      vector<size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

      map<KeyFrame *, int> mConnectedKeyFrameWeights;
      vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
      vector<int> mvOrderedWeights;

      // Spanning Tree and Loop Edges
      bool mbFirstConnection;
      KeyFrame * mpParent;
      set<KeyFrame *> mspChildrens;
      set<KeyFrame *> mspLoopEdges;

      // Bad flags
      bool mbNotErase; //server-only
      bool mbToBeErased; //server-only
      bool mbBad;

      mutex mMutexPose;
      mutex mMutexConnections;
      mutex mMutexFeatures;

   private:
      struct Header
      {
         id_type mnId;
         double mTimestamp;
         int N;
         int mnScaleLevels;
         float mfScaleFactor;
         float mfLogScaleFactor;
         bool mbFirstConnection;
         id_type parentKeyFrameId;
         bool mbBad;
      };

      struct KeyFrameWeight
      {
         id_type keyFrameId;
         int weight;
      };

      id_type mnId;

      double mTimestamp;

      int mN;

      // Vector of KeyPoints (features) based on original image(s). Used for visualization.
      vector<cv::KeyPoint> mvKeys;

      vector<cv::KeyPoint> mvKeysUn;

      vector<float> mvuRight;

      vector<float> mvDepth;

      cv::Mat mDescriptors;

      cv::Mat mTcp;

      int mnScaleLevels;
      float mfScaleFactor;
      float mfLogScaleFactor;
      vector<float> mvScaleFactors;
      vector<float> mvLevelSigma2;
      vector<float> mvInvLevelSigma2;

      // Grid (to speed up feature matching)
      const int mnGridCols;
      const int mnGridRows;

      atomic_bool mModified;

      static id_type PeekId(const void * data);

      static void * ReadMapPointIds(
         void * const buffer, 
         const Map & rMap, 
         unordered_map<id_type, MapPoint *> & newMapPoints, 
         vector<MapPoint *> & mpv);

      static void * WriteMapPointIds(void * const buffer, const vector<MapPoint *> & mpv);

      static void * ReadKeyFrameWeights(
         void * const buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames, 
         map<KeyFrame *, int> & kfWeights);
    
      static void * WriteKeyFrameWeights(void * const buffer, const map<KeyFrame *, int> & kfWeights);

      static void * ReadKeyFrameIds(
         void * const buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         vector<KeyFrame *> & kfv);

      static void * WriteKeyFrameIds(void * const buffer, const vector<KeyFrame *> & kfv);

      static void * ReadKeyFrameIds(
         void * const buffer, 
         const Map & rMap, 
         unordered_map<id_type, KeyFrame *> & newKeyFrames,
         set<KeyFrame *> & kfs);

      static void * WriteKeyFrameIds(void * const buffer, const set<KeyFrame *> & kfs);

      bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

      void AssignFeaturesToGrid();

      void PrintPrefix(ostream & out);

};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
