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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "Map.h"
#include "Enums.h"
#include "SyncPrint.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

   class Tracking;

   class FrameDrawer : SyncPrint
   {
   public:
      FrameDrawer(cv::FileStorage & settings);

      void Reset();

      // Update info from the last processed frame.
      void Update(
         bool onlyTracking,
         TrackingState lastProcessedState,
         cv::Mat & imGray, 
         std::vector<cv::KeyPoint> & initialKeys, 
         vector<int> & initialMatches,
         std::vector<cv::KeyPoint> & currentKeys, 
         std::vector<MapPoint *> & currentMapPoints, 
         std::vector<bool> & outliers,
         int keyFramesInMap,
         int mapPointsInMap
      );

      // Draw last processed frame.
      cv::Mat DrawFrame();

      int GetImageHeight();

      int GetImageWidth();

      int GetFrameHeight();

      int GetFrameWidth();

   protected:

      void DrawTextInfo(cv::Mat &im, TrackingState state, cv::Mat &imText);

      // Info of the frame to be drawn
      cv::Mat mIm;
      int N;
      vector<cv::KeyPoint> mvCurrentKeys;
      vector<bool> mvbMap, mvbVO;
      bool mbOnlyTracking;
      int mnTracked, mnTrackedVO;
      vector<cv::KeyPoint> mvIniKeys;
      vector<int> mvIniMatches;
      TrackingState mState;

      std::mutex mMutex;

   private:
      // 1/fps in ms
      double mT;
      int mImageWidth, mImageHeight, mTextInfoHeight, mnKFs, mnMPs;

      stringstream StateToString(TrackingState state);

   };

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
