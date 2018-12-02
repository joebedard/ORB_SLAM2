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


#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

#include <Enums.h>
#include <Tracking.h>
#include <ORBVocabulary.h>
#include <FrameDrawer.h>
#include <Mapper.h>
#include <MapperServer.h>
#include <SyncPrint.h>

using namespace ORB_SLAM2;

// logging variables
SyncPrint gOutMain("main: ");
SyncPrint gOutTrak("RunTracker: ");

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

void RunTracker(int threadId) try
{
   gOutTrak.Print("begin RunTracker");
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

   gOutTrak.Print("while (gShouldRun)");
   while (gShouldRun)
   {
      //gOutTrak.Print("rs2::frameset data = pipe.wait_for_frames();");
      rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
      //gOutTrak.Print("rs2::video_frame irFrame1 = data.get_infrared_frame(1);");
      rs2::video_frame irFrame1 = data.get_infrared_frame(1);
      //gOutTrak.Print("rs2::video_frame irFrame2 = data.get_infrared_frame(2);");
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
   gOutTrak.Print("end RunTracker");
}
catch( cv::Exception & e ) {
   string msg = string("cv::Exception: ") + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutMain.Print(msg);
}
catch (const rs2::error & e)
{
   string msg = string("RealSense error calling ") + e.get_failed_function() + "(" + e.get_failed_args() + "): " + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutMain.Print(msg);
}
catch (const exception& e)
{
   string msg = string("exception: ") + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutMain.Print(msg);
}
catch (...)
{
   string msg = string("There was an unknown exception in the main thread.");
   cerr << "RunTracker: " << msg << endl;
   gOutMain.Print(msg);
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

int RunViewer(
   vector<FrameDrawer *> & vFrameDrawers, 
   vector<MapDrawer *> & vMapDrawers, 
   vector<Tracking *> & vTrackers, 
   Mapper & mapper, 
   bool embeddedFrameDrawers) try
{
   //Initialize and start the Viewer thread
   Viewer viewer(vFrameDrawers, vMapDrawers, vTrackers, mapper, embeddedFrameDrawers);
   for (Tracking * tracker : vTrackers)
   {
      tracker->SetViewer(&viewer);
   }
   viewer.Run(); //ends when window is closed
   gShouldRun = false; //signal tracking threads to stop
   return EXIT_SUCCESS;
}
catch( cv::Exception & e ) {
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: cv::Exception: ") + e.what();
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (const exception & e)
{
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: exception: ") + e.what();
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (...)
{
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: There was an unknown exception in RunViewer");
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}

int main(int paramc, char * paramv[]) try
{
   FrameDrawer * pFrameDrawer = NULL;
   Tracking * pTracker = NULL;

   vector<FrameDrawer *> vFrameDrawers;
   vector<MapDrawer *> vMapDrawers;
   vector<Mapper *> vMapperClients;
   vector<Tracking *> vTrackers;

   ParseParams(paramc, paramv);

   //Load ORB Vocabulary
   SyncPrint::Print(NULL, "Loading ORB Vocabulary. This could take a while...");
   ORBVocabulary vocab;
   bool bVocLoad = vocab.loadFromFile(gVocabFilename);
   if (!bVocLoad)
   {
      SyncPrint::Print("Failed to open vocabulary file at: ", gVocabFilename);
      exit(-1);
   }
   SyncPrint::Print(NULL, "Vocabulary loaded!");

   cv::FileStorage mapperSettings(gMapperFilename, cv::FileStorage::READ);
   VerifySettings(mapperSettings, gMapperFilename);

   MapperServer mapperServer(vocab, false);
   vMapperClients.push_back(&mapperServer);
   MapDrawer mapDrawer(mapperSettings, mapperServer);
   vMapDrawers.push_back(&mapDrawer);

   for (int i = 0; i < TRACKER_QUANTITY; ++i)
   {
      cv::FileStorage trackerSettings(gTrackerFilename[i], cv::FileStorage::READ);
      string * pSerial = new string();
      VerifyTrackerSettings(trackerSettings, gTrackerFilename[i], *pSerial);
      pFrameDrawer = new FrameDrawer(trackerSettings);
      vFrameDrawers.push_back(pFrameDrawer);

      pTracker = new Tracking(trackerSettings, vocab, mapperServer, pFrameDrawer, NULL, SensorType::STEREO);
      vTrackers.push_back(pTracker);

      gThreadParams[i].serial = pSerial;
      gThreadParams[i].tracker = pTracker;
      gThreadParams[i].height = pFrameDrawer->GetImageHeight();
      gThreadParams[i].width = pFrameDrawer->GetImageWidth();
      gThreadParams[i].threadObj = new thread(RunTracker, i);
   }

   int returnCode = RunViewer(vFrameDrawers, vMapDrawers, vTrackers, mapperServer, true);

   // join threads and check return codes
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

   return returnCode;
}
catch( cv::Exception & e ) {
   string msg = string("cv::Exception: ") + e.what();
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (const rs2::error & e)
{
   string msg = string("RealSense error calling ") + e.get_failed_function() + "(" + e.get_failed_args() + "): " + e.what();
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (const exception & e)
{
   string msg = string("exception: ") + e.what();
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (...)
{
   string msg = string("There was an unknown exception in the main thread.");
   cerr << "main: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
