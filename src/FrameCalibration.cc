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

namespace ORB_SLAM2
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
      const cv::Mat & _K,
      const cv::Mat & _distCoef,
      const int & _width,
      const int & _height,
      const float & _blfx,
      const float & _thDepth) : FrameCalibration()
   {
      Initialize(_K, _distCoef, _width, _height, _blfx, _thDepth);
   }

   void FrameCalibration::Initialize(
      const cv::Mat & _K, 
      const cv::Mat & _distCoef, 
      const int & _width, 
      const int & _height, 
      const float & _blfx,
      const float & _thDepth)
   {
      if (_distCoef.at<float>(0) != 0.0)
      {
         cv::Mat mat(4, 2, CV_32F);
         mat.at<float>(0, 0) = 0.0; mat.at<float>(0, 1) = 0.0;
         mat.at<float>(1, 0) = _width; mat.at<float>(1, 1) = 0.0;
         mat.at<float>(2, 0) = 0.0; mat.at<float>(2, 1) = _height;
         mat.at<float>(3, 0) = _width; mat.at<float>(3, 1) = _height;

         // Undistort corners
         mat = mat.reshape(2);
         cv::undistortPoints(mat, mat, _K, _distCoef, cv::Mat(), _K);
         mat = mat.reshape(1);

         mMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
         mMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
         mMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
         mMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
      }
      else
      {
         mMinX = 0.0f;
         mMaxX = _width;
         mMinY = 0.0f;
         mMaxY = _height;
      }

      mWidth = _width;
      mHeight = _height;
      mK = _K;
      mDistCoef = _distCoef;
      mFx = _K.at<float>(0, 0);
      mFy = _K.at<float>(1, 1);
      mCx = _K.at<float>(0, 2);
      mCy = _K.at<float>(1, 2);
      mInvfx = 1.0f / mFx;
      mInvfy = 1.0f / mFy;
      mGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mMaxX - mMinX);
      mGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mMaxY - mMinY);
      mBlfx = _blfx;
      mBl = _blfx / mFx;
      mThDepth = _thDepth;
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
      int _width = pHeader->mWidth;
      int _height = pHeader->mHeight;
      float _bl = pHeader->mBl;
      float _thDepth = pHeader->mThDepth;

      // read variable-length data
      cv::Mat _K, _distCoef;
      void * pData = pHeader + 1;
      pData = Serializer::ReadMatrix(pData, _K);
      pData = Serializer::ReadMatrix(pData, _distCoef);

      Initialize(_K, _distCoef, _width, _height, _bl, _thDepth);

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