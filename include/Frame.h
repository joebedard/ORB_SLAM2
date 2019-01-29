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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "MapPoint.h"
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "FrameCalibration.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2_TEAM
{

   class MapPoint;
   class KeyFrame;

   class Frame
   {
   public:

      Frame::Frame();

      // Copy constructor.
      Frame(const Frame &frame);

      // Constructor for stereo cameras.
      Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, FrameCalibration * FC);

      // Constructor for RGB-D cameras.
      Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor, FrameCalibration * FC);

      // Constructor for Monocular cameras.
      Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, FrameCalibration * FC);

      // Extract ORB on the image. 0 for left image and 1 for right image.
      void ExtractORBLeft(const cv::Mat &im);
      void ExtractORBRight(const cv::Mat &im);

      // Compute Bag of Words representation.
      void ComputeBoW(ORBVocabulary & vocab);

      // Set the camera pose.
      void SetPose(cv::Mat Tcw);

      // Computes rotation, translation and camera center matrices from the camera pose.
      void UpdatePoseMatrices();

      // Returns the camera center.
      inline cv::Mat GetCameraCenter() {
         return mOw.clone();
      }

      // Returns inverse of rotation
      inline cv::Mat GetRotationInverse() {
         return mRwc.clone();
      }

      // Check if a MapPoint is in the frustum of the camera
      // and fill variables of the MapPoint to be used by the tracking
      bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

      // Compute the cell of a keypoint (return false if outside the grid)
      bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

      vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel = -1, const int maxLevel = -1) const;

      // Search a match for each keypoint in the left image to a keypoint in the right image.
      // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
      void ComputeStereoMatches();

      // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
      void ComputeStereoFromRGBD(const cv::Mat &imDepth);

      // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
      cv::Mat UnprojectStereo(const int &i);

   public:

      // Feature extractor. The right is used only in the stereo case.
      ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

      // Frame timestamp.
      double mTimeStamp;

      // Calibration matrix and OpenCV distortion parameters.
      FrameCalibration * mFC;

      // Number of KeyPoints (features).
      size_t N;

      // Vector of KeyPoints (features) based on original image(s). Used for visualization.
      std::vector<cv::KeyPoint> mvKeys, mvKeysRight;

      // Vector of undistorted KeyPoints (features). Used by tracking and mapping.
      // If it is a stereo frame, mvKeysUn is redundant because images are pre-rectified.
      // If it is a RGB-D frame, the RGB images might be distorted.
      std::vector<cv::KeyPoint> mvKeysUn;

      // Corresponding stereo coordinate for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      std::vector<float> mvuRight;

      // Corresponding depth for each KeyPoint.
      // If this frame is monocular, all elements are negative.
      std::vector<float> mvDepth;

      // Bag of Words Vector structures.
      DBoW2::BowVector mBowVec;
      DBoW2::FeatureVector mFeatVec;

      // ORB descriptor, each row associated to a keypoint.
      cv::Mat mDescriptors, mDescriptorsRight;

      // MapPoints associated to KeyPoints (via the index), NULL pointer if no association.
      // Each non-null element corresponds to an element in mvKeysUn.
      std::vector<MapPoint*> mvpMapPoints;

      // Flag to identify outlier associations.
      std::vector<bool> mvbOutlier;

      // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
      std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

      // Camera pose.
      cv::Mat mTcw;

      // Current and Next Frame id.
      static long unsigned int nNextId;
      long unsigned int mnId;

      // Reference Keyframe.
      KeyFrame* mpReferenceKF;

      // Scale pyramid info.
      int mnScaleLevels;
      float mfScaleFactor;
      float mfLogScaleFactor;
      vector<float> mvScaleFactors;
      vector<float> mvInvScaleFactors;
      vector<float> mvLevelSigma2;
      vector<float> mvInvLevelSigma2;


   private:

      // Undistort keypoints given OpenCV distortion parameters.
      // Only for the RGB-D case. Stereo must be already rectified!
      // (called in the constructor).
      void UndistortKeyPoints();

      // Assign keypoints to the grid for speed up feature matching (called in the constructor).
      void AssignFeaturesToGrid();

      // Rotation, translation and camera center
      cv::Mat mRcw;
      cv::Mat mtcw;
      cv::Mat mRwc;
      cv::Mat mOw; //==mtwc

   };

}// namespace ORB_SLAM

#endif // FRAME_H
