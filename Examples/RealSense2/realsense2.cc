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

using namespace std;

char * strVocFile = NULL;
char * strSettingsFile = NULL;

void parseArgs(int argc, char * argv[])
{
   if (argc != 3)
   {
      //cerr << endl << "Usage: ./realsense2 path_to_vocabulary_file path_to_settings_file" << endl;
      const char * usage = "Usage: ./realsense2 path_to_vocabulary_file path_to_settings_file";
      std::exception e(usage);
   }
   strVocFile = argv[1];
   strSettingsFile = argv[2];
}

int main(int argc, char * argv[]) try
{
   parseArgs(argc, argv);

   // Create SLAM system. It initializes all system threads and gets ready to process frames.
   ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

   // Vector for tracking time statistics
   vector<float> vTimesTrack;
   int ni = 0;
   vTimesTrack.resize(10000);
   std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

   cout << endl << "-------" << endl;
   cout << "Start processing sequence ..." << endl;

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   rs2::pipeline pipe;

   // Start streaming with default recommended configuration
   pipe.start();

   using namespace cv;
   const auto window_name = "Display Image";
   namedWindow(window_name, WINDOW_AUTOSIZE);

   while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
   {
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      rs2::video_frame irFrame0 = data.get_infrared_frame(0);
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);

      // Query frame size (width and height)
      const int w0 = irFrame0.get_width();
      const int h0 = irFrame0.get_height();
      const int w1 = irFrame1.get_width();
      const int h1 = irFrame1.get_height();

      // Create OpenCV matrix of size (w,h) from the colorized depth data
      Mat image0(Size(w0, h0), CV_8UC3, (void*)irFrame0.get_data(), Mat::AUTO_STEP);
      Mat image1(Size(w1, h1), CV_8UC3, (void*)irFrame1.get_data(), Mat::AUTO_STEP);

      // Update the window with new data
      //imshow(window_name, image);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

      double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - tStart).count();
      // Pass the images to the SLAM system
      SLAM.TrackStereo(image0, image1, tframe);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      vTimesTrack[ni++] = ttrack;

      // Wait to load the next frame
      /*double T = 0;
      if (ni<nImages - 1)
         T = vTimestamps[ni + 1] - tframe;
      else if (ni>0)
         T = tframe - vTimestamps[ni - 1];

      if (ttrack<T)
         sleep((T - ttrack)*1e6);*/
   }

   // Stop all threads
   SLAM.Shutdown();

   // Tracking time statistics
   sort(vTimesTrack.begin(), vTimesTrack.end());
   float totaltime = 0;
   for (int i = 0; i<ni; i++)
   {
      totaltime += vTimesTrack[i];
   }
   cout << "-------" << endl << endl;
   cout << "median tracking time: " << vTimesTrack[ni / 2] << endl;
   cout << "mean tracking time: " << totaltime / ni << endl;

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