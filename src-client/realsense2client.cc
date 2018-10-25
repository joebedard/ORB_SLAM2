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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

#include <Sleep.h>
#include <Enums.h>
#include <Tracking.h>
#include <ORBVocabulary.h>
#include <FrameDrawer.h>
#include <MapDrawer.h>
#include <Map.h>
#include <Mapper.h>
#include <MapperClient.h>
#include <MapperServer.h>
#include <SyncPrint.h>

using namespace ORB_SLAM2;

struct ThreadParam
{
   int returnCode;
   string * serial;
   Tracking * tracker;
   int height;
   int width;
   vector<float> timesTrack;
};
bool gShouldRun = true;

// command line parameters
char * gVocabFilename = NULL;
char * gMapperFilename = NULL;
char * gTrackerFilename = NULL;

void ParseParams(int paramc, char * paramv[])
{
   if (paramc != 4)
   {
      const char * usage = "Usage: ./realsense2client vocabulary_file_and_path mapper_settings_file_and_path tracker_settings_file_and_path";
      exception e(usage);
      throw e;
   }
   gVocabFilename = paramv[1];
   gMapperFilename = paramv[2];
   gTrackerFilename = paramv[3];
}

void VerifySettings(cv::FileStorage & settings, const char * settingsFilePath)
{
   if (!settings.isOpened())
   {
      std::string m("Failed to open settings file at: ");
      m.append(settingsFilePath);
      throw exception(m.c_str());
   }
}

void VerifyTrackerSettings(cv::FileStorage & settings, const char * settingsFilePath, string & serial)
{
   VerifySettings(settings, settingsFilePath);

   serial.append(settings["Camera.serial"]);
   if (0 == serial.length())
      throw exception("Camera.serial property is not set or value is not in quotes.");

}

void Print(const char * s)
{
   SyncPrint::Print("RunTracker: ", s);
}

void RunTracker(void * param) try
{
   Print("begin RunTracker");
   ThreadParam * threadParam = (ThreadParam *)param;
   int height = threadParam->height;
   int width = threadParam->width;

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   rs2::pipeline pipe;  //ok to create more than one pipe in different threads?

                        // create and resolve custom configuration for RealSense
   rs2::config customConfig;
   customConfig.enable_device(*threadParam->serial);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
   if (!customConfig.can_resolve(pipe))
   {
      stringstream ss;
      ss << "Can not resolve RealSense config for camera with serial number " << threadParam->serial;
      throw exception(ss.str().c_str());
   }

   rs2::pipeline_profile profile = pipe.start(customConfig);
   rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();

   // disable the projected IR pattern when using stereo IR images
   sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

   std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

   Print("while (gShouldRun)");
   while (gShouldRun)
   {
      Print("rs2::frameset data = pipe.wait_for_frames();");
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      Print("rs2::video_frame irFrame1 = data.get_infrared_frame(1);");
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);
      Print("rs2::video_frame irFrame2 = data.get_infrared_frame(2);");
      rs2::video_frame irFrame2 = data.get_infrared_frame(2);

      // Create OpenCV matrix of size (width, height)
      cv::Mat irMat1(cv::Size(width, height), CV_8UC1, (void*)irFrame1.get_data(), cv::Mat::AUTO_STEP);
      cv::Mat irMat2(cv::Size(width, height), CV_8UC1, (void*)irFrame2.get_data(), cv::Mat::AUTO_STEP);

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - tStart).count();

      threadParam->tracker->GrabImageStereo(irMat1, irMat2, tframe);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

      threadParam->timesTrack.push_back(ttrack);
   }

   threadParam->returnCode = EXIT_SUCCESS;
   Print("end RunTracker");
}
catch (const rs2::error & e)
{
   SyncPrint::Print("RealSense error calling ", e.get_failed_function() + "(" + e.get_failed_args() + "):\n    " + e.what());
   ThreadParam * threadParam = (ThreadParam *)param;
   threadParam->returnCode = EXIT_FAILURE;
}
catch (const std::exception & e)
{
   SyncPrint::Print("Exception in RunTracker: ", e.what());
   ThreadParam * threadParam = (ThreadParam *)param;
   threadParam->returnCode = EXIT_FAILURE;
}
catch (...)
{
   SyncPrint::Print(NULL, "An exception was not caught in RunTracker ");
   ThreadParam * threadParam = (ThreadParam *)param;
   threadParam->returnCode = EXIT_FAILURE;
}

void printStatistics(vector<float> & vTimesTrack)
{
   if (vTimesTrack.size() > 0)
   {
      // calculate time statistics
      sort(vTimesTrack.begin(), vTimesTrack.end());
      float totaltime = 0;
      for (int i = 0; i < vTimesTrack.size(); i++)
      {
         totaltime += vTimesTrack[i];
      }

      stringstream ss;
      ss << "tracking time statistics for client:\n";
      ss << "   median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << "\n";
      ss << "   mean tracking time: " << totaltime / vTimesTrack.size() << "\n";
      SyncPrint::Print(NULL, ss);
   }
}

int main(int paramc, char * paramv[]) try
{
   ParseParams(paramc, paramv);

   cv::FileStorage mapDrawSettings(gMapperFilename, cv::FileStorage::READ);
   VerifySettings(mapDrawSettings, gMapperFilename);

   cv::FileStorage trackerSettings(gTrackerFilename, cv::FileStorage::READ);
   string serial;
   VerifyTrackerSettings(trackerSettings, gTrackerFilename, serial);

   //Load ORB Vocabulary
   SyncPrint::Print(NULL, "Loading ORB Vocabulary. This could take a while...");
   ORBVocabulary vocab;
   bool bVocLoad = vocab.loadFromTextFile(gVocabFilename);
   if (!bVocLoad)
   {
      SyncPrint::Print("Failed to open vocabulary file at: ", gVocabFilename);
      exit(-1);
   }
   SyncPrint::Print(NULL, "Vocabulary loaded!");

   MapperServer mapperServer(vocab, false);
   MapperClient mapperClient(mapperServer, vocab, false);
   MapDrawer mapDrawer(mapDrawSettings, mapperClient);
   FrameDrawer frameDrawer(trackerSettings);
   Tracking tracker(trackerSettings, vocab, mapperClient, &frameDrawer, NULL, eSensor::STEREO);

   ThreadParam threadParam;
   threadParam.serial = &serial;
   threadParam.tracker = &tracker;
   threadParam.height = frameDrawer.GetImageHeight();
   threadParam.width = frameDrawer.GetImageWidth();
   thread trackerThread(RunTracker, &threadParam);

   {
      //Initialize and start the Viewer thread
      Viewer viewer(&frameDrawer, &mapDrawer, &tracker, &mapperServer);
      tracker.SetViewer(&viewer);
      viewer.Run(); //ends when window is closed
      gShouldRun = false; //signal tracking threads to stop
      // implicit Viewer destruction
   }

   trackerThread.join();

   printStatistics(threadParam.timesTrack);

   return threadParam.returnCode;
}
catch (const rs2::error & e)
{
   cout << "RealSense error calling " << e.get_failed_function() + "(" + e.get_failed_args() + "):\n    " + e.what();
   return EXIT_FAILURE;
}
catch (const exception& e)
{
   cout << "Exception in main thread: " << e.what();
   return EXIT_FAILURE;
}
catch (...)
{
   cout << "There was an unknown exception in the main thread.";
   return EXIT_FAILURE;
}
