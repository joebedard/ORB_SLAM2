/**
* This file is part of ORB-SLAM2-TEAM.
*
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


#ifndef FRAMECALIBRATION_H
#define FRAMECALIBRATION_H

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

   class FrameCalibration
   {
   public:

      // camera calibration

      const cv::Mat & K;
      const cv::Mat & distCoef;
      const float & fx;
      const float & fy;
      const float & cx;
      const float & cy;
      const float & invfx;
      const float & invfy;


      // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints

      const float & gridElementWidthInv;
      const float & gridElementHeightInv;


      // Undistorted Image Bounds

      const float & minX;
      const float & maxX;
      const float & minY;
      const float & maxY;


      // Stereo baseline multiplied by fx.
      const float & blfx;

      // Stereo baseline in meters.
      const float & bl;

      // Threshold close/far points
      // Points seen as close by the stereo/RGBD sensor are considered reliable
      // and inserted from just one frame. Far points requiere a match in two keyframes.
      const float & thDepth;

      FrameCalibration();

      FrameCalibration(const FrameCalibration & FC);

      FrameCalibration(
         const cv::Mat & K,
         const cv::Mat & distCoef,
         const int & width,
         const int & height,
         const float & bl,
         const float & thDepth);

      void Initialize(
         const cv::Mat & K,
         const cv::Mat & distCoef,
         const int & width,
         const int & height,
         const float & bl,
         const float & thDepth);

      size_t GetBufferSize() const;
      void * ReadBytes(void * const buffer);
      void * WriteBytes(void * const buffer) const;


   private:
      int mWidth;
      int mHeight;
      cv::Mat mK;
      cv::Mat mDistCoef;
      float mFx;
      float mFy;
      float mCx;
      float mCy;
      float mInvfx;
      float mInvfy;
      float mGridElementWidthInv;
      float mGridElementHeightInv;
      float mMinX;
      float mMaxX;
      float mMinY;
      float mMaxY;
      float mBlfx;
      float mBl;
      float mThDepth;

      struct Header
      {
         int mWidth;
         int mHeight;
         float mBl;
         float mThDepth;
      };

   };

} // namespace ORB_SLAM2

#endif // FRAMECALIBRATION_H