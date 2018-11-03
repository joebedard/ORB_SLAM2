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


#include "FrameCalibration.h"
#include <algorithm>

namespace ORB_SLAM2
{

   using namespace std;

   FrameCalibration::FrameCalibration(cv::Mat &K, cv::Mat &distCoef, ImageBounds &imageBounds)
      : K(K)
      , distCoef(distCoef)
      , fx(K.at<float>(0, 0))
      , fy(K.at<float>(1, 1))
      , cx(K.at<float>(0, 2))
      , cy(K.at<float>(1, 2))
      , invfx(1.0f / fx)
      , invfy(1.0f / fy)
      , minX(imageBounds.minX)
      , maxX(imageBounds.maxX)
      , minY(imageBounds.minY)
      , maxY(imageBounds.maxY)
      , gridElementWidthInv(static_cast<float>(FRAME_GRID_COLS) / (imageBounds.maxX - imageBounds.minX))
      , gridElementHeightInv(static_cast<float>(FRAME_GRID_ROWS) / (imageBounds.maxY - imageBounds.minY))
   {

   }

   FrameCalibration::ImageBounds::ImageBounds(cv::Mat &K, cv::Mat &distCoef, int width, int height)
   {
      if (distCoef.at<float>(0) != 0.0)
      {
         cv::Mat mat(4, 2, CV_32F);
         mat.at<float>(0, 0) = 0.0; mat.at<float>(0, 1) = 0.0;
         mat.at<float>(1, 0) = width; mat.at<float>(1, 1) = 0.0;
         mat.at<float>(2, 0) = 0.0; mat.at<float>(2, 1) = height;
         mat.at<float>(3, 0) = width; mat.at<float>(3, 1) = height;

         // Undistort corners
         mat = mat.reshape(2);
         cv::undistortPoints(mat, mat, K, distCoef, cv::Mat(), K);
         mat = mat.reshape(1);

         minX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
         maxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
         minY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
         maxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));

      }
      else
      {
         minX = 0.0f;
         maxX = width;
         minY = 0.0f;
         maxY = height;
      }
   }

}