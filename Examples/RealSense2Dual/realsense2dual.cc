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

#include <MapperClient.h>
#include <Sleep.h>
#include <Enums.h>
#include <Tracking.h>
#include <ORBVocabulary.h>
#include <FrameDrawer.h>
#include <Map.h>
#include <Mapper.h>
#include <MapperServer.h>
#include <SyncPrint.h>

using namespace ORB_SLAM2;

const int TRACKER_QUANTITY = 2;
struct ThreadParam
{
   int returnCode;
   string * serial;
   Tracking * tracker;
   int height;
   int width;
   vector<float> timesTrack;
   thread * threadObj;
};
ThreadParam gThreadParams[TRACKER_QUANTITY];
bool gShouldRun = true;

// command line parameters
char * gVocabFilename = NULL;
char * gMapperFilename = NULL;
char * gTrackerFilename[TRACKER_QUANTITY];

void ParseParams(int paramc, char * paramv[])
{
   if (paramc != 5)
   {
      const char * usage = "Usage: ./realsense2dual vocabulary_file_and_path mapper_settings_file_and_path tracker1_settings_file_and_path tracker2_settings_file_and_path";
      exception e(usage);
      throw e;
   }

   gVocabFilename = paramv[1];

   gMapperFilename = paramv[2];

   for (int i = 0; i < TRACKER_QUANTITY; i++)
   {
      gTrackerFilename[i] = paramv[i + 3];
   }
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

void Print(int threadId, const char * s)
{
   /*stringstream ss; ss << "RunTracker" << threadId << ": ";
   SyncPrint::Print(ss.str().c_str(), s);*/
}

void RunTracker(int threadId) try
{
   Print(threadId, "begin RunTracker");
   int height = gThreadParams[threadId].height;
   int width = gThreadParams[threadId].width;

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   rs2::pipeline pipe;  //ok to create more than one pipe in different threads?

   // create and resolve custom configuration for RealSense
   rs2::config customConfig;
   customConfig.enable_device(*gThreadParams[threadId].serial);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
   if (!customConfig.can_resolve(pipe))
   {
      stringstream ss;
      ss << "Can not resolve RealSense config for camera with serial number " << gThreadParams[threadId].serial;
      throw exception(ss.str().c_str());
   }

   rs2::pipeline_profile profile = pipe.start(customConfig);
   rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();

   // disable the projected IR pattern when using stereo IR images
   sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

   std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

   Print(threadId, "while (gShouldRun)");
   while (gShouldRun)
   {
      Print(threadId, "rs2::frameset data = pipe.wait_for_frames();");
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      Print(threadId, "rs2::video_frame irFrame1 = data.get_infrared_frame(1);");
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);
      Print(threadId, "rs2::video_frame irFrame2 = data.get_infrared_frame(2);");
      rs2::video_frame irFrame2 = data.get_infrared_frame(2);

      // Create OpenCV matrix of size (width, height)
      cv::Mat irMat1(cv::Size(width, height), CV_8UC1, (void*)irFrame1.get_data(), cv::Mat::AUTO_STEP);
      cv::Mat irMat2(cv::Size(width, height), CV_8UC1, (void*)irFrame2.get_data(), cv::Mat::AUTO_STEP);

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - tStart).count();

      gThreadParams[threadId].tracker->GrabImageStereo(irMat1, irMat2, tframe);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

      gThreadParams[threadId].timesTrack.push_back(ttrack);
   }

   gThreadParams[threadId].returnCode = EXIT_SUCCESS;
   Print(threadId, "end RunTracker");
}
catch (const rs2::error & e)
{
   SyncPrint::Print("RealSense error calling ", e.get_failed_function() + "(" + e.get_failed_args() + "):\n    " + e.what());
   gThreadParams[threadId].returnCode = EXIT_FAILURE;
}
catch (const exception& e)
{
   SyncPrint::Print("Exception in RunTracker: ", e.what());
   gThreadParams[threadId].returnCode = EXIT_FAILURE;
}
catch (...)
{
   SyncPrint::Print("An exception was not caught in RunTracker ", to_string(threadId));
   gThreadParams[threadId].returnCode = EXIT_FAILURE;
}

void printStatistics()
{
   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      vector<float> & vTimesTrack = gThreadParams[i].timesTrack;

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
         ss << "tracking time statistics for thread " << i << ":\n";
         ss << "   median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << "\n";
         ss << "   mean tracking time: " << totaltime / vTimesTrack.size() << "\n";
         SyncPrint::Print(NULL, ss);
      }
   }
}

int main(int paramc, char * paramv[]) try
{
   const bool SINGLE_MAPPER_CLIENT = true;

   MapDrawer * pMapDrawer = NULL;
   FrameDrawer * pFrameDrawer = NULL;
   MapperClient * pMapperClient = NULL;
   Tracking * pTracker = NULL;

   vector<FrameDrawer *> vFrameDrawers;
   vector<MapDrawer *> vMapDrawers;
   vector<MapperClient *> vMapperClients;
   vector<Tracking *> vTrackers;

   ParseParams(paramc, paramv);

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
   cv::FileStorage mapperSettings(gMapperFilename, cv::FileStorage::READ);
   VerifySettings(mapperSettings, gMapperFilename);

   if (SINGLE_MAPPER_CLIENT)
   {
      pMapperClient = new MapperClient(mapperSettings, vocab, false);
      vMapperClients.push_back(pMapperClient);
      pMapDrawer = new MapDrawer(mapperSettings, *pMapperClient);
      vMapDrawers.push_back(pMapDrawer);
   }

   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      cv::FileStorage trackerSettings(gTrackerFilename[i], cv::FileStorage::READ);
      string * pSerial = new string();
      VerifyTrackerSettings(trackerSettings, gTrackerFilename[i], *pSerial);
      pFrameDrawer = new FrameDrawer(trackerSettings);
      vFrameDrawers.push_back(pFrameDrawer);

      if (SINGLE_MAPPER_CLIENT)
      {
         pTracker = new Tracking(trackerSettings, vocab, *pMapperClient, pFrameDrawer, NULL, eSensor::STEREO);
      }
      else
      {
         pMapperClient = new MapperClient(mapperSettings, vocab, false);
         vMapperClients.push_back(pMapperClient);
         pMapDrawer = new MapDrawer(mapperSettings, *pMapperClient);
         vMapDrawers.push_back(pMapDrawer);
         pTracker = new Tracking(trackerSettings, vocab, *pMapperClient, pFrameDrawer, pMapDrawer, eSensor::STEREO);
      }
      vTrackers.push_back(pTracker);

      gThreadParams[i].serial = pSerial;
      gThreadParams[i].tracker = pTracker;
      gThreadParams[i].height = pFrameDrawer->GetImageHeight();
      gThreadParams[i].width = pFrameDrawer->GetImageWidth();
      gThreadParams[i].threadObj = new thread(RunTracker, i);
   }

   {
      //Initialize and start the Viewer thread
      Viewer viewer(vFrameDrawers, vMapDrawers, vTrackers, &mapperServer);
      for (Tracking * tracker : vTrackers)
      {
         tracker->SetViewer(&viewer);
      }
      viewer.Run(); //ends when window is closed
      gShouldRun = false; //signal tracking threads to stop
      // implicit Viewer destruction
   }

   // join threads and check return codes
   int returnCode = EXIT_SUCCESS;
   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      gThreadParams[i].threadObj->join();
      if (gThreadParams[i].returnCode == EXIT_FAILURE)
         returnCode = EXIT_FAILURE;
   }

   printStatistics();

   // destroy objects
   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      delete gThreadParams[i].threadObj;
      delete gThreadParams[i].tracker;
      delete gThreadParams[i].serial;
      delete vFrameDrawers[i];
   }

   for (FrameDrawer * pFD : vFrameDrawers)
   {
      delete pFD;
   }

   for (MapDrawer * pMD : vMapDrawers)
   {
      delete pMD;
   }

   for (MapperClient * pMC : vMapperClients)
   {
      delete pMC;
   }

   return returnCode;
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
