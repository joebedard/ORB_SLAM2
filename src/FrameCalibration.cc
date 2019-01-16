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
#include "Serializer.h"
#include <algorithm>

namespace ORB_SLAM2_TEAM
{

   using namespace std;

   FrameCalibration::FrameCalibration()
      : K(mK)
      , distCoef(mDistCoef)
      , fx(mFx)
      , fy(mFy)
      , cx(mCx)
      , cy(mCy)
      , invfx(mInvfx)
      , invfy(mInvfy)
      , minX(mMinX)
      , maxX(mMaxX)
      , minY(mMinY)
      , maxY(mMaxY)
      , gridElementWidthInv(mGridElementWidthInv)
      , gridElementHeightInv(mGridElementHeightInv)
      , blfx(mBlfx)
      , bl(mBl)
      , thDepth(mThDepth)
   {
   }

   FrameCalibration::FrameCalibration(const FrameCalibration & FC) : FrameCalibration()
   {
      Initialize(FC.mK, FC.mDistCoef, FC.mWidth, FC.mHeight, FC.mBlfx, FC.mThDepth);
   }

   FrameCalibration::FrameCalibration(
      const cv::Mat & K,
      const cv::Mat & distCoef,
      const int & width,
      const int & height,
      const float & blfx,
      const float & thDepth) : FrameCalibration()
   {
      Initialize(K, distCoef, width, height, blfx, thDepth);
   }

   void FrameCalibration::Initialize(
      const cv::Mat & K, 
      const cv::Mat & distCoef, 
      const int & width, 
      const int & height, 
      const float & blfx,
      const float & thDepth)
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

         mMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
         mMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
         mMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
         mMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
      }
      else
      {
         mMinX = 0.0f;
         mMaxX = width;
         mMinY = 0.0f;
         mMaxY = height;
      }

      mWidth = width;
      mHeight = height;
      mK = K;
      mDistCoef = distCoef;
      mFx = K.at<float>(0, 0);
      mFy = K.at<float>(1, 1);
      mCx = K.at<float>(0, 2);
      mCy = K.at<float>(1, 2);
      mInvfx = 1.0f / mFx;
      mInvfy = 1.0f / mFy;
      mGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mMaxX - mMinX);
      mGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mMaxY - mMinY);
      mBlfx = blfx;
      mBl = blfx / mFx;
      mThDepth = thDepth;
   }

   size_t FrameCalibration::GetBufferSize() const
   {
      size_t size = sizeof(FrameCalibration::Header);
      size += Serializer::GetMatBufferSize(mK);
      size += Serializer::GetMatBufferSize(mDistCoef);
      return size;
   }

   void * FrameCalibration::ReadBytes(void * const buffer)
   {
      FrameCalibration::Header * pHeader = (FrameCalibration::Header *)buffer;
      int width = pHeader->mWidth;
      int height = pHeader->mHeight;
      float bl = pHeader->mBl;
      float thDepth = pHeader->mThDepth;

      // read variable-length data
      cv::Mat K, distCoef;
      void * pData = pHeader + 1;
      pData = Serializer::ReadMatrix(pData, K);
      pData = Serializer::ReadMatrix(pData, distCoef);

      Initialize(K, distCoef, width, height, bl, thDepth);

      return pData;
   }

   void * FrameCalibration::WriteBytes(void * const buffer) const
   {
      FrameCalibration::Header * pHeader = (FrameCalibration::Header *)buffer;
      pHeader->mWidth = mWidth;
      pHeader->mHeight = mHeight;
      pHeader->mBl = mBl;
      pHeader->mThDepth = mThDepth;

      // write variable-length data
      void * pData = pHeader + 1;
      pData = Serializer::WriteMatrix(pData, mK);
      pData = Serializer::WriteMatrix(pData, mDistCoef);
      return pData;
   }

}