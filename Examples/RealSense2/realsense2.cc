/**
* This file is part of ORB-SLAM2-CS.
*
* Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
* For more information see <https://github.com/joebedard/ORB_SLAM2_CS>
*
* ORB-SLAM2-CS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2-CS is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2-CS. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

#include<System.h>
#include "Sleep.h"
#include "Enums.h"

using namespace std;

char * strVocFile = NULL;
char * strSettingsFile = NULL;

void parseArgs(int argc, char * argv[])
{
   if (argc != 3)
   {
      const char * usage = "Usage: ./realsense2 vocabulary_file_and_path settings_file_and_path";
      std::exception e(usage);
      throw e;
   }
   strVocFile = argv[1];
   strSettingsFile = argv[2];
}

cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const cv::Mat depthMat)
{
   using namespace cv;
   using namespace rs2;

   Mat dm;
   depthMat.convertTo(dm, CV_64F);
   auto depth_scale = pipe.get_active_profile()
      .get_device()
      .first<depth_sensor>()
      .get_depth_scale();
   dm = dm * depth_scale;
   return dm;
}

int main(int argc, char * argv[]) try
{
   parseArgs(argc, argv);

   //Check settings file
   cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
   if (!fsSettings.isOpened())
   {
      cerr << "Failed to open settings file at: " << strSettingsFile << endl;
      return EXIT_FAILURE;
   }

   int width = fsSettings["Camera.width"];
   int height = fsSettings["Camera.height"];

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   rs2::pipeline pipe;

   // create and resolve custom configuration for RealSense
   rs2::config customConfig;
   customConfig.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
   if (!customConfig.can_resolve(pipe))
   {
      cout << "can not resolve RealSense config" << endl;
      return EXIT_FAILURE;
   }

   // Create SLAM system. It initializes all system threads and gets ready to process frames.
   ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::STEREO, true);

   // Vector for tracking time statistics
   vector<float> vTimesTrack;
   std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

   cout << endl << "-------" << endl;
   cout << "Start processing streams ..." << endl;

   // start RealSense streaming
   rs2::pipeline_profile profile = pipe.start(customConfig);

   // disable the projected IR pattern when using stereo IR images
   rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();
   sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

   using namespace cv;
   while (!SLAM.IsQuitting())
   {
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);
      rs2::video_frame irFrame2 = data.get_infrared_frame(2);

      // Create OpenCV matrix of size (width, height)
      Mat irMat1(Size(width, height), CV_8UC1, (void*)irFrame1.get_data(), Mat::AUTO_STEP);
      Mat irMat2(Size(width, height), CV_8UC1, (void*)irFrame2.get_data(), Mat::AUTO_STEP);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

      double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - tStart).count();

      // Pass the images to the SLAM system
      SLAM.TrackStereo(irMat1, irMat2, tframe);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      vTimesTrack.push_back(ttrack);
   }

   // Stop all threads
   SLAM.Shutdown();

   // Tracking time statistics
   sort(vTimesTrack.begin(), vTimesTrack.end());
   float totaltime = 0;
   for (int i = 0; i<vTimesTrack.size(); i++)
   {
      totaltime += vTimesTrack[i];
   }
   cout << "-------" << endl << endl;
   cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
   cout << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;

   // Save camera trajectory
   SLAM.SaveTrajectoryKITTI("RealSense2CameraTrajectory.txt");

   return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
   std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
   return EXIT_FAILURE;
}
catch (const std::exception& e)
{
   std::cerr << e.what() << std::endl;
   return EXIT_FAILURE;
}