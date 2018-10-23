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

#include "FrameDrawer.h"
#include "MapPoint.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

   FrameDrawer::FrameDrawer(cv::FileStorage & fSettings) :
      SyncPrint("FrameDrawer: "), mnKFs(0), mnMPs(0),
      N(0), mState(NO_IMAGES_YET), mbOnlyTracking(false)
   {
      float fps = fSettings["Camera.fps"];
      if (fps < 1.0f)
         throw new exception("Camera.fps is not set.");
      mT = 1e3 / fps;

      mImageWidth = (int)fSettings["Camera.width"];
      if (0 == mImageWidth)
         throw new exception("Camera.width is not set.");

      mImageHeight = (int)fSettings["Camera.height"];
      if (0 == mImageHeight)
         throw new exception("Camera.height is not set.");

      stringstream s = StateToString(mState);
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      mTextInfoHeight = textSize.height + 10;

      mIm = cv::Mat(GetFrameHeight(), GetFrameWidth(), CV_8UC3, cv::Scalar(0, 0, 0));
   }

   void FrameDrawer::Reset()
   {
      unique_lock<mutex> lock(mMutex);

      mnKFs = 0;
      mnMPs = 0;
      N = 0;
      mState = NO_IMAGES_YET;
      mbOnlyTracking = false;

      mIm = cv::Mat(GetFrameHeight(), GetFrameWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

      mvCurrentKeys.clear();
      mvbVO.clear();
      mvbMap.clear();
      mvIniKeys.clear();
      mvIniMatches.clear();
   }

   cv::Mat FrameDrawer::DrawFrame()
   {
      cv::Mat im;
      vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
      vector<int> vMatches; // Initialization: correspondeces with reference keypoints
      vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
      vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
      eTrackingState state; // Tracking state

      //Copy variables within scoped mutex
      {
         unique_lock<mutex> lock(mMutex);
         state = mState;

         mIm.copyTo(im);

         if (mState == NOT_INITIALIZED)
         {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
         }
         else if (mState == TRACKING_OK)
         {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
         }
         else if (mState == TRACKING_LOST)
         {
            vCurrentKeys = mvCurrentKeys;
         }
      } // destroy scoped mutex -> release mutex

      if (im.channels() < 3) //this should be always true
         cvtColor(im, im, CV_GRAY2BGR);

      //Draw
      if (state == NOT_INITIALIZED) //INITIALIZING
      {
         for (unsigned int i = 0; i < vMatches.size(); i++)
         {
            if (vMatches[i] >= 0)
            {
               cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
                  cv::Scalar(0, 255, 0));
            }
         }
      }
      else if (state == TRACKING_OK)
      {
         mnTracked = 0;
         mnTrackedVO = 0;
         const float r = 5;
         const int n = vCurrentKeys.size();
         for (int i = 0;i < n;i++)
         {
            if (vbVO[i] || vbMap[i])
            {
               cv::Point2f pt1, pt2;
               pt1.x = vCurrentKeys[i].pt.x - r;
               pt1.y = vCurrentKeys[i].pt.y - r;
               pt2.x = vCurrentKeys[i].pt.x + r;
               pt2.y = vCurrentKeys[i].pt.y + r;

               // This is a match to a MapPoint in the map
               if (vbMap[i])
               {
                  cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                  cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                  mnTracked++;
               }
               else // This is match to a "visual odometry" MapPoint created in the last frame
               {
                  cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                  cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
                  mnTrackedVO++;
               }
            }
         }
      }

      cv::Mat imWithInfo;
      DrawTextInfo(im, state, imWithInfo);

      return imWithInfo;
   }

   int FrameDrawer::GetImageHeight()
   {
      return mImageHeight;
   }

   int FrameDrawer::GetImageWidth()
   {
      return mImageWidth;
   }

   int FrameDrawer::GetFrameHeight()
   {
      return mImageHeight + mTextInfoHeight;
   }

   int FrameDrawer::GetFrameWidth()
   {
      return mImageWidth;
   }

   stringstream FrameDrawer::StateToString(eTrackingState state)
   {
      stringstream s;
      if (state == NO_IMAGES_YET)
         s << " WAITING FOR IMAGES";
      else if (state == NOT_INITIALIZED)
         s << " TRYING TO INITIALIZE ";
      else if (state == TRACKING_OK)
      {
         if (!mbOnlyTracking)
            s << "SLAM MODE |  ";
         else
            s << "LOCALIZATION | ";
         s << "KFs: " << mnKFs << ", MPs: " << mnMPs << ", Matches: " << mnTracked;
         if (mnTrackedVO > 0)
            s << ", + VO matches: " << mnTrackedVO;
      }
      else if (state == TRACKING_LOST)
      {
         s << " TRACK LOST. TRYING TO RELOCALIZE ";
      }
      return s;
   }

   void FrameDrawer::DrawTextInfo(cv::Mat &im, eTrackingState state, cv::Mat &imText)
   {
      stringstream s = StateToString(state);
      imText = cv::Mat(im.rows + mTextInfoHeight, im.cols, im.type());
      im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
      imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(mTextInfoHeight, im.cols, im.type());
      cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
   }

   void FrameDrawer::Update(Tracking & tracker, Map & map)
   {
      Print("begin Update");
      unique_lock<mutex> lock(mMutex);
      tracker.mImGray.copyTo(mIm);
      mvCurrentKeys = tracker.mCurrentFrame.mvKeys;
      N = mvCurrentKeys.size();
      mvbVO = vector<bool>(N, false);
      mvbMap = vector<bool>(N, false);
      mbOnlyTracking = tracker.mbOnlyTracking;

      mnKFs = map.KeyFramesInMap();
      mnMPs = map.MapPointsInMap();

      if (tracker.mLastProcessedState == NOT_INITIALIZED)
      {
         mvIniKeys = tracker.mInitialFrame.mvKeys;
         mvIniMatches = tracker.mvIniMatches;
      }
      else if (tracker.mLastProcessedState == TRACKING_OK)
      {
         for (int i = 0;i < N;i++)
         {
            MapPoint* pMP = tracker.mCurrentFrame.mvpMapPoints[i];
            if (pMP)
            {
               if (!tracker.mCurrentFrame.mvbOutlier[i])
               {
                  if (pMP->Observations() > 0)
                     mvbMap[i] = true;
                  else
                     mvbVO[i] = true;
               }
            }
         }
      }
      mState = tracker.mLastProcessedState;
      Print("end Update");
   }

} //namespace ORB_SLAM
