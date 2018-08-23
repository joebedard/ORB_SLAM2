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

#include <Sleep.h>
#include <Enums.h>
#include <Tracking.h>
#include <ORBVocabulary.h>
#include <FrameDrawer.h>
#include <Map.h>

using namespace std;
using namespace ORB_SLAM2;
using namespace cv;

const int TRACKER_QUANTITY = 2;
struct ThreadParam
{
   int returnCode;
   string * serial;
   Tracking * tracker;
   int height;
   int width;
   mutex * mutexOutput;
   vector<float> timesTrack;
   thread * threadObj;
};
ThreadParam mThreadParams[TRACKER_QUANTITY];
bool mShouldRun = true;

// command line parameters
char * mVocFile = NULL;

void ParseParams(int paramc, char * paramv[])
{
   if (paramc != 4)
   {
      const char * usage = "Usage: ./realsense2 vocabulary_file_and_path first_settings_file_and_path second_settings_file_and_path";
      exception e(usage);
      throw e;
   }
   mVocFile = paramv[1];
}

void ParseSettings(FileStorage & settings, const char * settingsFilePath, string & serial)
{
   if (!settings.isOpened())
   {
      std::string m("Failed to open settings file at: ");
      m.append(settingsFilePath);
      throw exception(m.c_str());
   }

   serial.append(settings["Camera.serial"]);
   if (0 == serial.length())
      throw exception("Camera.serial property is not set or value is not in quotes.");

}

void printDebug(int threadId, mutex * mutexPrint, const char * message)
{
   if (threadId == 0) return;
   unique_lock<mutex> lock(*mutexPrint);
   std::cerr << message << std::endl;
}

void RunTracker(int threadId) try
{
   int height = mThreadParams[threadId].height;
   int width = mThreadParams[threadId].width;

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   rs2::pipeline pipe;  //ok to create more than one pipe in different threads?

   // create and resolve custom configuration for RealSense
   rs2::config customConfig;
   customConfig.enable_device(*mThreadParams[threadId].serial);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
   customConfig.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
   if (!customConfig.can_resolve(pipe))
   {
      stringstream ss;
      ss << "Can not resolve RealSense config for camera with serial number " << mThreadParams[threadId].serial;
      throw exception(ss.str().c_str());
   }
    
   rs2::pipeline_profile profile = pipe.start(customConfig);
   rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();

   // disable the projected IR pattern when using stereo IR images
   sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

   std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

   while (mShouldRun)
   {
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);
      rs2::video_frame irFrame2 = data.get_infrared_frame(2);

      // Create OpenCV matrix of size (width, height)
      Mat irMat1(Size(width, height), CV_8UC1, (void*)irFrame1.get_data(), Mat::AUTO_STEP);
      Mat irMat2(Size(width, height), CV_8UC1, (void*)irFrame2.get_data(), Mat::AUTO_STEP);

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - tStart).count();

      mThreadParams[threadId].tracker->GrabImageStereo(irMat1, irMat2, tframe);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      mThreadParams[threadId].timesTrack.push_back(ttrack);
   }

   mThreadParams[threadId].returnCode = EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
   unique_lock<mutex> lock(*mThreadParams[threadId].mutexOutput);
   std::cerr << std::endl << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
   mThreadParams[threadId].returnCode = EXIT_FAILURE;
}
catch (const exception& e)
{
   unique_lock<mutex> lock(*mThreadParams[threadId].mutexOutput);
   std::cerr << std::endl << e.what() << std::endl;
   mThreadParams[threadId].returnCode = EXIT_FAILURE;
}
catch (...)
{
   std::cerr << std::endl << "An exception was not caught in thread " << threadId << "." << std::endl;
   mThreadParams[threadId].returnCode = EXIT_FAILURE;
}

void printStatistics()
{
   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      vector<float> & vTimesTrack = mThreadParams[i].timesTrack;

      if (vTimesTrack.size() > 0)
      {
         // calculate time statistics
         sort(vTimesTrack.begin(), vTimesTrack.end());
         float totaltime = 0;
         for (int i = 0; i<vTimesTrack.size(); i++)
         {
            totaltime += vTimesTrack[i];
         }

         cout << "tracking time statistics for thread " << i << ":" << endl;
         cout << "   median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
         cout << "   mean tracking time: " << totaltime / vTimesTrack.size() << endl;
      }
   }
}

int main(int paramc, char * paramv[]) try
{
   ParseParams(paramc, paramv);

   //Load ORB Vocabulary
   cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
   ORBVocabulary * pVocab = new ORBVocabulary();
   bool bVocLoad = pVocab->loadFromTextFile(mVocFile);
   if (!bVocLoad)
   {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Failed to open at: " << mVocFile << endl;
      exit(-1);
   }
   cout << "Vocabulary loaded!" << endl << endl;

   //Create the Map
   Map * pMap = new Map();

   //Create mutex for output synchronization
   mutex mutexOutput;

   //Initialize the Mapper
   Mapper * pMapper = new Mapper(&mutexOutput, pMap, pVocab, false);

   vector<FrameDrawer *> vFrameDrawers;
   vector<MapDrawer *> vMapDrawers;
   vector<Tracking *> vTrackers;

   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      FileStorage settings(paramv[i + 2], FileStorage::READ);

      string * pSerial = new string();
      ParseSettings(settings, paramv[i + 2], *pSerial);

      FrameDrawer * frameDrawer = new FrameDrawer(pMap, settings);
      vFrameDrawers.push_back(frameDrawer);

      MapDrawer * mapDrawer = new MapDrawer(&mutexOutput, pMap, settings);
      vMapDrawers.push_back(mapDrawer);

      Tracking * tracker = new Tracking(&mutexOutput, pVocab, frameDrawer, mapDrawer, pMapper, settings, eSensor::STEREO);
      vTrackers.push_back(tracker);

      mThreadParams[i].serial = pSerial;
      mThreadParams[i].tracker = tracker;
      mThreadParams[i].height = frameDrawer->GetImageHeight();
      mThreadParams[i].width = frameDrawer->GetImageWidth();
      mThreadParams[i].mutexOutput = &mutexOutput;
      mThreadParams[i].threadObj = new thread(RunTracker, i);
   }

   //Initialize and start the Viewer thread
   Viewer viewer(&mutexOutput, vFrameDrawers, vMapDrawers, vTrackers);
   for (auto tracker : vTrackers)
   {
       tracker->SetViewer(&viewer);
   }
   viewer.Run();
   mShouldRun = false; //signal tracking threads to stop

   int returnCode = EXIT_SUCCESS;
   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      mThreadParams[i].threadObj->join();
      if (mThreadParams[i].returnCode == EXIT_FAILURE)
         returnCode = EXIT_FAILURE;
      delete mThreadParams[i].threadObj;
   }

   printStatistics();

   return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
   std::cerr << std::endl << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
   return EXIT_FAILURE;
}
catch (const exception& e)
{
   std::cerr << std::endl << e.what() << std::endl;
   return EXIT_FAILURE;
}
catch (...)
{
   std::cerr << std::endl << "An exception was not caught in the main thread." << std::endl;
   return EXIT_FAILURE;
}
