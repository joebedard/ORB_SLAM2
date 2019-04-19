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


#include <chrono>
#include <opencv2/core/core.hpp>

#include <Duration.h>
#include <Sleep.h>
#include <Enums.h>
#include <Tracking.h>
#include <ORBVocabulary.h>
#include <FrameDrawer.h>
#include <Mapper.h>
#include <MapperServer.h>
#include <SyncPrint.h>
#include <fstream>

using namespace ORB_SLAM2_TEAM;

// logging variables
SyncPrint gOutMain("main: ");
SyncPrint gOutTrak("RunTracker: ");

struct ThreadParam
{
   int returnCode;
   Tracking * tracker;
   int height;
   int width;
   thread * threadObj;
};
vector<ThreadParam> gThreadParams;
bool gShouldRun = true;

// master config settings
string gVocabFileName;
string gMapperFileName;
bool gNoViewer = false;
string gMetricsLog;
size_t gTrackerQuantity = 0;
vector<string> gTrackerFileName;
vector<string> gTrackerLeftImageFileName;
vector<string> gTrackerRightImageFileName;
vector<string> gRunningPoseLog;
vector<string> gFinalPoseLog;

void Log(const char * prefix, const char * message)
{
   SyncPrint::Print(prefix, message);
   if (prefix)
      clog << prefix;
   if (message)
      clog << message;
   if (prefix || message)
      clog << endl;
}

void Log(const char * prefix, string & message)
{
   Log(prefix, message.c_str());
}

void VerifyString(const string & name, const string & value)
{
   if (0 == value.length())
   {
      string m = name + " is not set or is not in quotes.";
      throw exception(m.c_str());
   }
}

void ParseParams(int paramc, char * paramv[])
{
   if (paramc != 2)
   {
      const char * usage = "Usage: ./stereo_far_team master_configuration_file_and_path";
      exception e(usage);
      throw e;
   }

   cv::FileStorage config(paramv[1], cv::FileStorage::READ);
   if (!config.isOpened())
   {
      std::string m("Failed to open settings file at: ");
      m.append(paramv[1]);
      throw exception(m.c_str());
   }

   gVocabFileName = config["Vocabulary"];
   VerifyString("Vocabulary file name", gVocabFileName);

   gMapperFileName = config["Mapper"];
   VerifyString("Mapper file name", gMapperFileName);

   gNoViewer = (int)config["NoViewer"] > 0 ? true : false;

   gMetricsLog = config["MetricsLog"];
   VerifyString("Metrics Log file name", gMetricsLog);

   gTrackerQuantity = (int)config["Tracker.Quantity"];
   if (0 >= gTrackerQuantity)
      throw exception("Tracker.Quantity must be 1 or more.");

   gTrackerFileName.resize(gTrackerQuantity);
   gTrackerLeftImageFileName.resize(gTrackerQuantity);
   gTrackerRightImageFileName.resize(gTrackerQuantity);
   gRunningPoseLog.resize(gTrackerQuantity);
   gFinalPoseLog.resize(gTrackerQuantity);
   for (int i = 0; i < gTrackerQuantity; i++) 
   {
      string paramNum = to_string(i + 1);

      string paramName = string("Tracker.Settings.") + paramNum;
      gTrackerFileName[i] = config[paramName];
      VerifyString(paramName, gTrackerFileName[i]);

      paramName = string("Tracker.LeftImages.") + paramNum;
      gTrackerLeftImageFileName[i] = config[paramName];
      VerifyString(paramName, gTrackerLeftImageFileName[i]);

      paramName = string("Tracker.RightImages.") + paramNum;
      gTrackerRightImageFileName[i] = config[paramName];
      VerifyString(paramName, gTrackerRightImageFileName[i]);

      paramName = string("Tracker.RunningPoseLog.") + paramNum;
      gRunningPoseLog[i] = config[paramName];
      VerifyString(paramName, gRunningPoseLog[i]);

      paramName = string("Tracker.FinalPoseLog.") + paramNum;
      gFinalPoseLog[i] = config[paramName];
      VerifyString(paramName, gFinalPoseLog[i]);
   }
}

void VerifySettings(cv::FileStorage & settings, const string & settingsFilePath)
{
   if (!settings.isOpened())
   {
      std::string m("Failed to open settings file at: ");
      m.append(settingsFilePath);
      throw exception(m.c_str());
   }
}

void LoadImageNames(const string & strPathFile,
   vector<unsigned long long> & vTimestamp, vector<string> & vstrImage)
{
   ifstream fFile;
   fFile.open(strPathFile.c_str());
   vTimestamp.reserve(5000);
   vstrImage.reserve(5000);
   size_t pos = strPathFile.rfind("/");
   string strPath = strPathFile.substr(0, pos+1);

   string s;
   unsigned long long t;
   string name;
   while(!fFile.eof())
   {
      getline(fFile, s);
      if(!s.empty())
      {
         stringstream ss;
         ss << s;
         ss >> t;
         ss >> name;
         vTimestamp.push_back(t);
         vstrImage.push_back(strPath + name.substr(1, name.length()-2));
      }
   }
}

void RunTracker(int threadId) try
{
   gOutTrak.Print("begin RunTracker");
   gThreadParams[threadId].returnCode = EXIT_FAILURE;
   int height = gThreadParams[threadId].height;
   int width = gThreadParams[threadId].width;
   string imageLeftFileName = gTrackerLeftImageFileName[threadId];
   string imageRightFileName = gTrackerRightImageFileName[threadId];
   Tracking * pTracker = gThreadParams[threadId].tracker;

   // Retrieve paths to images
   vector<string> vstrImageFileNamesLeft;
   vector<string> vstrImageFileNamesRight;
   vector<unsigned long long> vTimestampsLeft, vTimestampsRight;
   LoadImageNames(imageLeftFileName, vTimestampsLeft, vstrImageFileNamesLeft);
   LoadImageNames(imageRightFileName, vTimestampsRight, vstrImageFileNamesRight);

   // Check consistency in the number of images and depthmaps
   if (vstrImageFileNamesLeft.empty())
   {
      throw exception("No images found in provided path for left camera.");
   }
   if (vstrImageFileNamesRight.empty())
   {
      throw exception("No images found in provided path for right camera.");
   }
   if (vstrImageFileNamesLeft.size() != vstrImageFileNamesRight.size())
   {
      throw exception("Different number of images for left and right cameras.");
   }

   const int nImages = vstrImageFileNamesLeft.size();

   // Main loop
   cv::Mat imLeft, imRight;
   for(int ni=0; ni < nImages; ni++)
   {
      time_type t1 = GetNow();

      // Read left and right images from file
      imLeft = cv::imread(vstrImageFileNamesLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
      imRight = cv::imread(vstrImageFileNamesRight[ni], CV_LOAD_IMAGE_UNCHANGED);
      unsigned long long tframe = vTimestampsLeft[ni];

      if(imLeft.empty())
      {
         string m = string("Failed to load image at: ") + vstrImageFileNamesLeft[ni];
         throw exception(m.c_str());
      }
      if(imRight.empty())
      {
         string m = string("Failed to load image at: ") + vstrImageFileNamesRight[ni];
         throw exception(m.c_str());
      }

      pTracker->GrabImageStereo(imLeft, imRight, tframe / 1e9);
      double ttrack = Duration(GetNow(), t1);

      // Wait to load the next frame
      unsigned long long T = 0;
      if (ni < nImages-1)
         T = vTimestampsLeft[ni+1] - tframe;
      else if (ni>0)
         T = tframe - vTimestampsLeft[ni-1];

      if (ttrack*1e6 < T/1e3)
         sleep(T/1e3 - ttrack*1e6);
   }

   gThreadParams[threadId].returnCode = EXIT_SUCCESS;
   gOutTrak.Print("end RunTracker");
}
catch (cv::Exception & e)
{
   string msg = string("cv::Exception: ") + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutTrak.Print(msg);
}
catch (const exception & e)
{
   string msg = string("exception: ") + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutTrak.Print(msg);
}
catch (...)
{
   string msg = string("There was an unknown exception in the main thread.");
   cerr << "RunTracker: " << msg << endl;
   gOutTrak.Print(msg);
}

void PrintStatistics(list<Statistics> & stats)
{
   stringstream ss;
   for (auto it = stats.begin(), endIt = stats.end(); it != endIt; it++)
   {
      ss << "   " << it->Name << " (" << it->N << ", " << it->Mean << ", " << it->SD << ")" << endl;
   }
   Log(NULL, ss.str());
}

void PrintMetrics(MapperServer & mapper)
{
   ofstream metricsFile;
   metricsFile.open(gMetricsLog);

   // statistics for the mapping thread
   mapper.Shutdown();
   mapper.WriteMetrics(metricsFile);
   Log(NULL, "mapping metrics:");
   PrintStatistics(mapper.GetStatistics());

   // metrics for each tracking thread
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      Tracking * pTracker = gThreadParams[i].tracker;
      pTracker->SaveRunningTrajectoryTUM(gRunningPoseLog[i]);
      pTracker->SaveFinalTrajectoryTUM(gFinalPoseLog[i]);
      pTracker->WriteMetrics(metricsFile);

      stringstream ss;
      ss << "tracking metrics for thread " << i << ": " << endl;
      for (auto it : pTracker->GetMetrics())
      {
         ss << "   " << it.first << ": " << it.second << endl;
      }
      Log(NULL, ss.str());
   }

   metricsFile.close();
}

int RunViewer(
   vector<FrameDrawer *> & vFrameDrawers, 
   MapDrawer * pMapDrawer, 
   vector<Tracking *> & vTrackers, 
   Mapper & mapper, 
   bool embeddedFrameDrawers) try
{
   //Initialize and start the Viewer thread
   Viewer viewer(vFrameDrawers, pMapDrawer, vTrackers, mapper, embeddedFrameDrawers);
   for (Tracking * tracker : vTrackers)
   {
      tracker->SetViewer(&viewer);
   }
   viewer.Run(); //ends when window is closed
   gShouldRun = false; //signal tracking threads to stop
   return EXIT_SUCCESS;
}
catch( cv::Exception & e )
{
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: cv::Exception: ") + e.what();
   cerr << "RunViewer: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (const exception & e)
{
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: exception: ") + e.what();
   cerr << "RunViewer: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}
catch (...)
{
   gShouldRun = false; //signal tracking threads to stop
   string msg = string("RunViewer: There was an unknown exception in RunViewer");
   cerr << "RunViewer: " << msg << endl;
   gOutMain.Print(msg);
   return EXIT_FAILURE;
}

int main(int paramc, char * paramv[]) try
{
   FrameDrawer * pFrameDrawer = NULL;
   Tracking * pTracker = NULL;

   vector<FrameDrawer *> vFrameDrawers;
   vector<Tracking *> vTrackers;

   ParseParams(paramc, paramv);

   //Load ORB Vocabulary
   Log(NULL, "Loading ORB Vocabulary...");
   ORBVocabulary vocab;
   bool bVocLoad = vocab.loadFromFile(gVocabFileName);
   if (!bVocLoad)
   {
      Log("Failed to open vocabulary file at: ", gVocabFileName);
      exit(-1);
   }
   Log(NULL, "Running ORB_SLAM2_TEAM...");

   cv::FileStorage mapperSettings(gMapperFileName, cv::FileStorage::READ);
   VerifySettings(mapperSettings, gMapperFileName);

   MapperServer mapperServer(vocab, false, gTrackerQuantity);
   MapDrawer mapDrawer(mapperSettings, mapperServer);

   gThreadParams.resize(gTrackerQuantity);
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      cv::FileStorage trackerSettings(gTrackerFileName[i], cv::FileStorage::READ);
      VerifySettings(trackerSettings, gTrackerFileName[i]);
      pFrameDrawer = new FrameDrawer(trackerSettings);
      vFrameDrawers.push_back(pFrameDrawer);

      pTracker = new Tracking(trackerSettings, vocab, mapperServer, pFrameDrawer, NULL, SensorType::STEREO);
      vTrackers.push_back(pTracker);

      gThreadParams[i].tracker = pTracker;
      gThreadParams[i].height = pFrameDrawer->GetImageHeight();
      gThreadParams[i].width = pFrameDrawer->GetImageWidth();
      gThreadParams[i].threadObj = new thread(RunTracker, i);
   }

   int returnCode = EXIT_SUCCESS;
   if (!gNoViewer)
      returnCode = RunViewer(vFrameDrawers, &mapDrawer, vTrackers, mapperServer, true);

   // join threads and check return codes
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      gThreadParams[i].threadObj->join();
      if (gThreadParams[i].returnCode == EXIT_FAILURE)
         returnCode = EXIT_FAILURE;
   }

   Log(NULL, "Data sequences completed...");
   PrintMetrics(mapperServer);

   // destroy objects
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      delete gThreadParams[i].threadObj;
      delete gThreadParams[i].tracker;
      delete vFrameDrawers[i];
   }

   return returnCode;
}
catch( cv::Exception & e ) 
{
   string msg = string("cv::Exception: ") + e.what();
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
