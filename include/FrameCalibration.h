/**
* This file is part of ORB-SLAM2-NET.
*
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

      class ImageBounds
      {
      public:
         float minX;
         float maxX;
         float minY;
         float maxY;

         ImageBounds(cv::Mat &K, cv::Mat &distCoef, int width, int height);
      };


      // camera calibration

      const cv::Mat K;
      const cv::Mat distCoef;
      const float fx;
      const float fy;
      const float cx;
      const float cy;
      const float invfx;
      const float invfy;


      // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints

      const float gridElementWidthInv;
      const float gridElementHeightInv;


      // Undistorted Image Bounds

      const float minX;
      const float maxX;
      const float minY;
      const float maxY;


      FrameCalibration(cv::Mat &K, cv::Mat &distCoef, ImageBounds &imageBounds);

   };

} // namespace ORB_SLAM2

#endif // FRAMECALIBRATION_H