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

using namespace ORB_SLAM2;

// logging variables
SyncPrint gOutMain("main: ");
SyncPrint gOutTrak("RunTracker: ");

struct ThreadParam
{
   int returnCode;
   Tracking * tracker;
   int height;
   int width;
   vector<float> timesTrack;
   thread * threadObj;
};
vector<ThreadParam> gThreadParams;
bool gShouldRun = true;

// master config settings
string gVocabFileName;
string gMapperFileName;
size_t gTrackerQuantity = 0;
vector<string> gTrackerFileName;
vector<string> gTrackerLeftImageDirName;
vector<string> gTrackerRightImageDirName;
vector<string> gTrackerTimestampsFileName;

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
      const char * usage = "Usage: ./stereo_euroc_team master_configuration_file_and_path";
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

   gTrackerQuantity = (int)config["Tracker.Quantity"];
   if (0 == gTrackerQuantity)
      throw exception("Tracker.Quantity must be 1 or more.");

   gTrackerFileName.resize(gTrackerQuantity);
   gTrackerLeftImageDirName.resize(gTrackerQuantity);
   gTrackerRightImageDirName.resize(gTrackerQuantity);
   gTrackerTimestampsFileName.resize(gTrackerQuantity);
   for (int i = 0; i < gTrackerQuantity; i++) 
   {
      string paramNum = to_string(i + 1);

      string paramName = string("Tracker.Settings.") + paramNum;
      gTrackerFileName[i] = config[paramName];
      VerifyString(paramName, gTrackerFileName[i]);

      paramName = string("Tracker.LeftImages.") + paramNum;
      gTrackerLeftImageDirName[i] = config[paramName];
      VerifyString(paramName, gTrackerLeftImageDirName[i]);

      paramName = string("Tracker.RightImages.") + paramNum;
      gTrackerRightImageDirName[i] = config[paramName];
      VerifyString(paramName, gTrackerRightImageDirName[i]);

      paramName = string("Tracker.Timestamps.") + paramNum;
      gTrackerTimestampsFileName[i] = config[paramName];
      VerifyString(paramName, gTrackerTimestampsFileName[i]);
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

void LoadImages(const string & strPathLeft, const string & strPathRight, const string & strPathTimes,
   vector<string> & vstrImageLeft, vector<string> & vstrImageRight, vector<double> & vTimeStamps)
{
   ifstream fTimes;
   fTimes.open(strPathTimes.c_str());
   vTimeStamps.reserve(5000);
   vstrImageLeft.reserve(5000);
   vstrImageRight.reserve(5000);
   while(!fTimes.eof())
   {
      string s;
      getline(fTimes,s);
      if(!s.empty())
      {
         stringstream ss;
         ss << s;
         vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
         vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
         double t;
         ss >> t;
         vTimeStamps.push_back(t/1e9);

      }
   }
}

void RunTracker(int threadId) try
{
   gOutTrak.Print("begin RunTracker");
   gThreadParams[threadId].returnCode = EXIT_FAILURE;
   int height = gThreadParams[threadId].height;
   int width = gThreadParams[threadId].width;
   string imageLeftDirName = gTrackerLeftImageDirName[threadId];
   string imageRightDirName = gTrackerRightImageDirName[threadId];
   string timestampsFileName = gTrackerTimestampsFileName[threadId];
   Tracking * pTracker = gThreadParams[threadId].tracker;

   // Retrieve paths to images
   vector<string> vstrImageFilenamesLeft;
   vector<string> vstrImageFilenamesRight;
   vector<double> vTimestamps;
   LoadImages(imageLeftDirName, imageRightDirName, timestampsFileName, vstrImageFilenamesLeft, vstrImageFilenamesRight, vTimestamps);

   // Check consistency in the number of images and depthmaps
   if (vstrImageFilenamesLeft.empty())
   {
      throw exception("No images found in provided path for left camera.");
   }
   if (vstrImageFilenamesRight.empty())
   {
      throw exception("No images found in provided path for right camera.");
   }
   if (vstrImageFilenamesLeft.size() != vstrImageFilenamesRight.size())
   {
      throw exception("Different number of images for left and right cameras.");
   }

   const int nImages = vstrImageFilenamesLeft.size();

   // Vector for tracking time statistics
   vector<float> & vTimesTrack = gThreadParams[threadId].timesTrack;
   vTimesTrack.resize(nImages);

   // Main loop
   cv::Mat imLeft, imRight;
   for(int ni=0; ni < nImages; ni++)
   {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageFilenamesLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
      imRight = cv::imread(vstrImageFilenamesRight[ni], CV_LOAD_IMAGE_UNCHANGED);
      double tframe = vTimestamps[ni];

      if(imLeft.empty())
      {
         string m = string("Failed to load image at: ") + vstrImageFilenamesLeft[ni];
         throw exception(m.c_str());
      }
      if(imRight.empty())
      {
         string m = string("Failed to load image at: ") + vstrImageFilenamesRight[ni];
         throw exception(m.c_str());
      }

      time_type t1 = GetNow();
      pTracker->GrabImageStereo(imLeft, imRight, tframe);
      double ttrack = Duration(GetNow(), t1);
      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0.0;
      if (ni < nImages-1)
         T = vTimestamps[ni+1] - tframe;
      else if (ni>0)
         T = tframe - vTimestamps[ni-1];

      if (ttrack < T)
         sleep((T-ttrack) * 1e6);
   }

   // Save camera trajectory
   //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
   //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

   gThreadParams[threadId].returnCode = EXIT_SUCCESS;
   gOutTrak.Print("end RunTracker");
}
catch (cv::Exception & e)
{
   string msg = string("cv::Exception: ") + e.what();
   cerr << "RunTracker: " << msg << endl;
   gOutMain.Print(msg);
}
catch (const exception & e)
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

void PrintStatistics()
{
   for (int i = 0; i < gTrackerQuantity; ++i)
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
         ss << "tracking time statistics for thread " << i << " : " << gTrackerTimestampsFileName[i] << endl;
         ss << "   median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
         ss << "   mean tracking time: " << totaltime / vTimesTrack.size() << endl;
         SyncPrint::Print(NULL, ss);
      }
   }
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
   vector<Mapper *> vMapperClients;
   vector<Tracking *> vTrackers;

   ParseParams(paramc, paramv);

   //Load ORB Vocabulary
   SyncPrint::Print(NULL, "Loading ORB Vocabulary. This could take a while...");
   ORBVocabulary vocab;
   bool bVocLoad = vocab.loadFromFile(gVocabFileName);
   if (!bVocLoad)
   {
      SyncPrint::Print("Failed to open vocabulary file at: ", gVocabFileName);
      exit(-1);
   }
   SyncPrint::Print(NULL, "Vocabulary loaded!");

   cv::FileStorage mapperSettings(gMapperFileName, cv::FileStorage::READ);
   VerifySettings(mapperSettings, gMapperFileName);

   MapperServer mapperServer(vocab, false, gTrackerQuantity);
   vMapperClients.push_back(&mapperServer);
   MapDrawer mapDrawer(mapperSettings, mapperServer);

   gThreadParams.resize(gTrackerQuantity);
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      cv::FileStorage trackerSettings(gTrackerFileName[i], cv::FileStorage::READ);
      string * pSerial = new string();
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

   int returnCode = RunViewer(vFrameDrawers, &mapDrawer, vTrackers, mapperServer, true);

   // join threads and check return codes
   for (int i = 0; i < gTrackerQuantity; ++i)
   {
      gThreadParams[i].threadObj->join();
      if (gThreadParams[i].returnCode == EXIT_FAILURE)
         returnCode = EXIT_FAILURE;
   }

   PrintStatistics();

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
