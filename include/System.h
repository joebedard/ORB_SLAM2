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


#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "Mapper.h"
#include "Enums.h"
#include "SyncPrint.h"

namespace ORB_SLAM2_TEAM
{

   class System : SyncPrint
   {
   public:

      // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
      System(const string &vocabFilename, const string &settingsFilename, const SensorType sensor, const bool bUseViewer = true);

      ~System();

      // Proccess the given stereo frame. Images must be synchronized and rectified.
      // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
      // Returns the camera pose (empty if tracking fails).
      cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

      // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
      // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
      // Input depthmap: Float (CV_32F).
      // Returns the camera pose (empty if tracking fails).
      cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

      // Proccess the given monocular frame
      // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
      // Returns the camera pose (empty if tracking fails).
      cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

      // Returns true if there have been a big map change (loop closure, global BA)
      // since last call to this function
      bool MapChanged();

      // Returns true if the user clicked the Viewer's Quit button
      // This should be checked after each call to Track*()
      bool IsQuitting();

      // All threads will be requested to finish.
      // It waits until all threads have finished.
      // This function must be called before saving the trajectory.
      void Shutdown();

      // Save camera trajectory in the TUM RGB-D dataset format.
      // Only for stereo and RGB-D. This method does not work for monocular.
      // Call first Shutdown()
      // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
      void SaveTrajectoryTUM(const string &filename);

      // Save keyframe poses in the TUM RGB-D dataset format.
      // This method works for all sensor input.
      // Call first Shutdown()
      // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
      void SaveKeyFrameTrajectoryTUM(const string &filename);

      // Save camera trajectory in the KITTI dataset format.
      // Only for stereo and RGB-D. This method does not work for monocular.
      // Call first Shutdown()
      // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
      void SaveTrajectoryKITTI(const string &filename);

      // TODO: Save/Load functions
      // SaveMap(const string &filename);
      // LoadMap(const string &filename);

   private:

      // Temporarily make these 3 functions private until they are really needed publicly
      // Information from most recent processed frame
      // You can call this right after TrackMonocular (or stereo or RGBD)
      int GetTrackingState();

      std::vector<MapPoint*> GetTrackedMapPoints();
      std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

      // Input sensor
      SensorType mSensor;

      // ORB vocabulary used for place recognition and feature matching.
      ORBVocabulary* mpVocabulary;

      // KeyFrame database for place recognition (relocalization and loop detection).
      KeyFrameDatabase* mpKeyFrameDatabase;

      // The Mapper encapsulates all mapping functionality of the system
      Mapper * mpMapper;

      // Tracker. It receives a frame and computes the associated camera pose.
      // It also decides when to insert a new keyframe, create some new MapPoints and
      // performs relocalization if tracking fails.
      Tracking* mpTracker;

      // The viewer draws the map and the current camera pose. It uses Pangolin.
      Viewer* mpViewer;

      FrameDrawer* mpFrameDrawer;
      MapDrawer * mpMapDrawer;

      // The Tracking thread "lives" in the main execution thread that creates the System object.
      std::thread* mptViewer;

      // Tracking state
      int mTrackingState;
      std::vector<MapPoint*> mTrackedMapPoints;
      std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
      std::mutex mMutexState;

   };

}// namespace ORB_SLAM

#endif // SYSTEM_H
