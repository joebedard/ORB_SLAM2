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

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

const cv::String cameraWindowLeft = "Left Camera";
const cv::String cameraWindowRight = "Right Camera";

int main(int argc, char * argv[]) try
{
   // Declare depth colorizer for pretty visualization of depth data
   rs2::colorizer color_map;

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
      rs2::frame depth = color_map(data.get_depth_frame());

      // Query frame size (width and height)
      const int w = depth.as<rs2::video_frame>().get_width();
      const int h = depth.as<rs2::video_frame>().get_height();

      // Create OpenCV matrix of size (w,h) from the colorized depth data
      Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

      // Update the window with new data
      imshow(window_name, image);
   }

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

int main2(int argc, char **argv)
{
   VideoCapture cap;
   // open the default camera, use something different from 0 otherwise;
   // Check VideoCapture documentation.
   if (!cap.open(0))
      return 0;
   for (;;)
   {
      Mat frame;
      cap >> frame;
      if (frame.empty()) break; // end of video stream
      imshow(cameraWindowLeft, frame);
      if (waitKey(10) == 27) break; // stop capturing by pressing ESC 
   }
   // the camera will be closed automatically upon exit
   // cap.close();
   return 0;
}