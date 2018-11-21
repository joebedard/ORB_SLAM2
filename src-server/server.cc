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

#include <iostream>
#include <conio.h>
#include <opencv2/core/core.hpp>
#include <zmq.hpp>
#include <SyncPrint.h>
#include <Sleep.h>
#include <Enums.h>
#include <Messages.h>
#include <ORBVocabulary.h>
#include <MapperServer.h>
#include <MapDrawer.h>
#include <Viewer.h>
#include <Serializer.h>

using namespace ORB_SLAM2;

/***
   ORB_SLAM2 Server with shared map for multiple tracking clients.
***/

// logging variables
SyncPrint gOutMain("main: ");
SyncPrint gOutServ("server: ");
SyncPrint gOutPub("publisher: ");

// command line parameters
char * gVocabFilename = NULL;
char * gMapperFilename = NULL;

// settings from config file
struct Settings
{
   std::string serverAddress;
   int serverTimeout;
   int serverLinger;
   std::string publisherAddress;
   int publisherTimeout;
   int publisherLinger;
};

// server variables
struct ServerParam
{
   int returnCode;
   zmq::socket_t * socket;
};
bool gShouldRun = true;
zmq::socket_t * gSocketPub;
MapperServer * gMapper = NULL;

void ParseParams(int paramc, char * paramv[])
{
   if (paramc != 3)
   {
      const char * usage = "Usage: ./server vocabulary_file_and_path mapper_settings_file_and_path";
      std::exception e(usage);
      throw e;
   }
   gVocabFilename = paramv[1];
   gMapperFilename = paramv[2];
}

void VerifySettings(cv::FileStorage & fileStorage, const char * settingsFilePath, Settings & settings)
{
   if (!fileStorage.isOpened())
   {
      std::string m("Failed to open file at: ");
      m.append(settingsFilePath);
      throw std::exception(m.c_str());
   }

   settings.serverAddress.append(fileStorage["Server.Address"]);
   if (0 == settings.serverAddress.length())
      throw std::exception("Server.Address property is not set or value is not in quotes.");

   settings.serverTimeout = fileStorage["Server.Timeout"];

   settings.serverLinger = fileStorage["Server.Linger"];

   settings.publisherAddress.append(fileStorage["Publisher.Address"]);
   if (0 == settings.publisherAddress.length())
      throw std::exception("Publisher.Address property is not set or value is not in quotes.");

   settings.publisherTimeout = fileStorage["Publisher.Timeout"];

   settings.publisherLinger = fileStorage["Publisher.Linger"];

}

zmq::message_t BuildReplyString(ReplyCode code, const char * str)
{
   size_t msgSize = sizeof(ReplyCode) + sizeof(char) * (strlen(str) + 1);
   zmq::message_t reply(msgSize);
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = code;
   strcpy(pRepData->message, str);
   return reply;
}

zmq::message_t HelloService(zmq::message_t & request)
{
   GreetRequest * pReqData = request.data<GreetRequest>();
   gOutServ.Print(string("Received ") + pReqData->message);
   if (0 == strcmp(pReqData->message, "Hello"))
   {
      gOutServ.Print("Replying World");
      return BuildReplyString(ReplyCode::SUCCEEDED, "World");
   }
   else
   {
      gOutServ.Print("Replying Hello");
      return BuildReplyString(ReplyCode::SUCCEEDED, "Hello");
   }
}

zmq::message_t LoginTracker(zmq::message_t & request)
{
   gOutServ.Print("begin LoginTracker");
   LoginTrackerRequest * pReqData = request.data<LoginTrackerRequest>();
   cv::Mat pivotCalib(4, 4, CV_32F);
   pivotCalib.at<float>(0, 0) = pReqData->pivotCalib[0];
   pivotCalib.at<float>(0, 1) = pReqData->pivotCalib[1];
   pivotCalib.at<float>(0, 2) = pReqData->pivotCalib[2];
   pivotCalib.at<float>(0, 3) = pReqData->pivotCalib[3];
   pivotCalib.at<float>(1, 0) = pReqData->pivotCalib[4];
   pivotCalib.at<float>(1, 1) = pReqData->pivotCalib[5];
   pivotCalib.at<float>(1, 2) = pReqData->pivotCalib[6];
   pivotCalib.at<float>(1, 3) = pReqData->pivotCalib[7];
   pivotCalib.at<float>(2, 0) = pReqData->pivotCalib[8];
   pivotCalib.at<float>(2, 1) = pReqData->pivotCalib[9];
   pivotCalib.at<float>(2, 2) = pReqData->pivotCalib[10];
   pivotCalib.at<float>(2, 3) = pReqData->pivotCalib[11];
   pivotCalib.at<float>(3, 0) = pReqData->pivotCalib[12];
   pivotCalib.at<float>(3, 1) = pReqData->pivotCalib[13];
   pivotCalib.at<float>(3, 2) = pReqData->pivotCalib[14];
   pivotCalib.at<float>(3, 3) = pReqData->pivotCalib[15];

   zmq::message_t reply(sizeof(LoginTrackerReply));
   LoginTrackerReply * pRepData = reply.data<LoginTrackerReply>();
   gMapper->LoginTracker(pivotCalib, 
      pRepData->trackerId, 
      pRepData->firstKeyFrameId, 
      pRepData->keyFrameIdSpan,
      pRepData->firstMapPointId, 
      pRepData->mapPointIdSpan);

   {
      // re-send pivot calibration via pub-sub to all tracking clients
      size_t msgSize = sizeof(UpdateTrackerMessage);
      msgSize += Serializer::GetMatBufferSize(pivotCalib);
      zmq::message_t message(msgSize);
      UpdateTrackerMessage * pMsgData = message.data<UpdateTrackerMessage>();
      pMsgData->subscribeId = -1; // all tracking clients
      pMsgData->messageId = MessageId::PIVOT_UPDATE;
      pMsgData->trackerId = pRepData->trackerId;
      Serializer::WriteMatrix(pMsgData + 1, pivotCalib);
      gSocketPub->send(message);
   }

   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end LoginTracker");
   return reply;
}

zmq::message_t LogoutTracker(zmq::message_t & request)
{
   gOutServ.Print("begin LogoutTracker");
   GeneralRequest * pReqData = request.data<GeneralRequest>();

   gMapper->LogoutTracker(pReqData->trackerId);

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end LogoutTracker");
   return reply;
}

zmq::message_t UpdatePose(zmq::message_t & request)
{
   gOutServ.Print("begin UpdatePose");
   GeneralRequest * pReqData = request.data<GeneralRequest>();
   void * pData = pReqData + 1;
   cv::Mat poseTcw;
   pData = Serializer::ReadMatrix(pData, poseTcw);

   gMapper->UpdatePose(pReqData->trackerId, poseTcw);

   {
      // re-send pose via pub-sub to all tracking clients
      size_t msgSize = sizeof(UpdateTrackerMessage);
      msgSize += Serializer::GetMatBufferSize(poseTcw);
      zmq::message_t message(msgSize);
      UpdateTrackerMessage * pMsgData = message.data<UpdateTrackerMessage>();
      pMsgData->subscribeId = -1; // all tracking clients
      pMsgData->messageId = MessageId::POSE_UPDATE;
      pMsgData->trackerId = pReqData->trackerId;
      Serializer::WriteMatrix(pMsgData + 1, poseTcw);
      gSocketPub->send(message);
   }

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end UpdatePose");
   return reply;
}

zmq::message_t InitializeMono(zmq::message_t & request)
{
   gOutServ.Print("begin InitializeMono");
   std::unordered_map<id_type, KeyFrame *> newKeyFrames;
   std::unordered_map<id_type, MapPoint *> newMapPoints;

   GeneralRequest * pReqData = request.data<GeneralRequest>();
   void * pData = pReqData + 1;

   // read KeyFrames
   KeyFrame * pKF1 = NULL;
   KeyFrame * pKF2 = NULL;
   pData = KeyFrame::Read(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, &pKF1);
   pData = KeyFrame::Read(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, &pKF2);
   if (newKeyFrames.size() != 2)
      throw exception("InitializeMono newKeyFrames.size() != 2");

   // read MapPoints 
   std::vector<MapPoint*> mapPoints;
   pData = MapPoint::ReadVector(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, mapPoints);
   if (newMapPoints.size() != mapPoints.size())
      throw exception("InitializeMono newMapPoints.size() != mapPoints.size()");

   try
   {
      gMapper->InitializeMono(pReqData->trackerId, mapPoints, pKF1, pKF2);
   }
   catch (exception & e)
   {
      delete pKF1;
      delete pKF2;
      for (MapPoint * pMP : mapPoints)
         delete pMP;
      throw e;
   }

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end InitializeMono");
   return reply;
}

zmq::message_t InitializeStereo(zmq::message_t & request)
{
   gOutServ.Print("begin InitializeStereo");
   std::unordered_map<id_type, KeyFrame *> newKeyFrames;
   std::unordered_map<id_type, MapPoint *> newMapPoints;

   GeneralRequest * pReqData = request.data<GeneralRequest>();
   void * pData = pReqData + 1;

   // read KeyFrame
   KeyFrame * pKF = NULL;
   pData = KeyFrame::Read(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, &pKF);
   if (newKeyFrames.size() != 1)
      throw exception("InitializeStereo newKeyFrames.size() != 1");
   
   // read MapPoints
   std::vector<MapPoint*> mapPoints;
   pData = MapPoint::ReadVector(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, mapPoints);
   if (newMapPoints.size() != mapPoints.size())
      throw exception("InitializeStereo newMapPoints.size() != mapPoints.size()");

   try
   {
      gMapper->InitializeStereo(pReqData->trackerId, mapPoints, pKF);
   }
   catch (exception & e)
   {
      delete pKF;
      for (MapPoint * pMP : mapPoints)
         delete pMP;
      throw e;
   }

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end InitializeStereo");
   return reply;
}

zmq::message_t GetMap(zmq::message_t & request)
{
   gOutServ.Print("begin GetMap");

   GeneralRequest * pReqData = request.data<GeneralRequest>();

   {
      // lock map and send it via pub-sub to the tracking client
      unique_lock<mutex> lock(gMapper->GetMutexMapUpdate());
      std::set<id_type> noDeletes; // no deleted MapPoints or KeyFrames
      MapChangeEvent mce;
      mce.updatedKeyFrames = gMapper->GetMap().GetKeyFrameSet();
      mce.deletedKeyFrames = noDeletes;
      mce.updatedMapPoints = gMapper->GetMap().GetMapPointSet();
      mce.deletedMapPoints = noDeletes;

      size_t msgSize = sizeof(GeneralMessage) + mce.GetBufferSize();
      zmq::message_t message(msgSize);
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      pMsgData->subscribeId = pReqData->trackerId;
      pMsgData->messageId = MessageId::MAP_CHANGE;
      void * pData = pMsgData + 1;
      pData = mce.WriteBytes(pData);
      gSocketPub->send(message);
   }

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end GetMap");
   return reply;
}

zmq::message_t InsertKeyFrame(zmq::message_t & request)
{
   gOutServ.Print("begin InsertKeyFrame");
   std::unordered_map<id_type, KeyFrame *> newKeyFrames;
   std::unordered_map<id_type, MapPoint *> newMapPoints;

   GeneralRequest * pReqData = request.data<GeneralRequest>();
   void * pData = pReqData + 1;

   // read KeyFrame
   KeyFrame * pKF = NULL;
   pData = KeyFrame::Read(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, &pKF);
   if (newKeyFrames.size() != 1)
      throw exception("InsertKeyFrame newKeyFrames.size() != 1");

   // read MapPoints
   std::vector<MapPoint*> mapPoints;
   pData = MapPoint::ReadVector(pData, gMapper->GetMap(), newKeyFrames, newMapPoints, mapPoints);
   if (newMapPoints.size() != mapPoints.size())
      throw exception("InsertKeyFrame newMapPoints.size() != mapPoints.size()");

   bool inserted = gMapper->InsertKeyFrame(pReqData->trackerId, mapPoints, pKF);
   if (!inserted)
   {
      delete pKF;
      for (MapPoint * pMP : mapPoints)
         delete pMP;
   }

   zmq::message_t reply(sizeof(InsertKeyFrameReply));
   InsertKeyFrameReply * pRepData = reply.data<InsertKeyFrameReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;
   pRepData->inserted = inserted;

   gOutServ.Print("end InsertKeyFrame");
   return reply;
}

zmq::message_t Reset(zmq::message_t & request)
{
   gOutServ.Print("begin Reset");

   gMapper->Reset();

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end Reset");
   return reply;
}

// array of function pointer
zmq::message_t (*gServices[ServiceId::quantityServiceId])(zmq::message_t & request) = {
   HelloService, LoginTracker, LogoutTracker, InitializeMono, InitializeStereo, GetMap, UpdatePose, InsertKeyFrame, Reset};

void RunServer(void * param) try
{
   ServerParam * serverParam = (ServerParam *)param;

   zmq::socket_t & socket = *serverParam->socket;

   zmq::message_t request;
   while (gShouldRun) 
   {
      if (socket.recv(&request, ZMQ_NOBLOCK))
      {
         try 
         {
            GeneralRequest * pReqData = request.data<GeneralRequest>();
            if (pReqData->serviceId < ServiceId::quantityServiceId)
            {
               zmq::message_t reply = gServices[pReqData->serviceId](request);
               socket.send(reply);
            }
            else
            {
               zmq::message_t reply = BuildReplyString(ReplyCode::UNKNOWN_SERVICE, "Unknown Service");
               socket.send(reply);
            }
         } 
         catch (std::exception & e)
         {
            zmq::message_t reply = BuildReplyString(ReplyCode::FAILED, e.what());
            socket.send(reply);
         }
      }
      else
      {
         sleep(1000);
      }
   }
   serverParam->returnCode = EXIT_SUCCESS;
}
catch (zmq::error_t & e)
{
   gOutServ.Print(string("error_t: ") + e.what());
   ServerParam * serverParam = (ServerParam *)param;
   serverParam->returnCode = EXIT_FAILURE;
}
catch (const std::exception & e)
{
   gOutServ.Print(string("exception: ") + e.what());
   ServerParam * serverParam = (ServerParam *)param;
   serverParam->returnCode = EXIT_FAILURE;
}
catch (...)
{
   gOutServ.Print("an exception was not caught in RunServer");
   ServerParam * serverParam = (ServerParam *)param;
   serverParam->returnCode = EXIT_FAILURE;
}

class PrivateMapperObserver : public MapperObserver
{
public:
   PrivateMapperObserver() {}

   virtual void HandleMapReset() try
   {
      zmq::message_t message(sizeof(GeneralMessage));
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      pMsgData->subscribeId = -1; // all trackers
      pMsgData->messageId = MessageId::MAP_RESET;
      gSocketPub->send(message);
   }
   catch (zmq::error_t & e)
   {
      gOutPub.Print(string("HandleMapReset error_t: ") + e.what());
   }
   catch (const std::exception & e)
   {
      gOutPub.Print(string("HandleMapReset exception: ") + e.what());
   }
   catch (...)
   {
      gOutPub.Print("HandleMapReset: an exception was not caught");
   }

   virtual void HandleMapChanged(MapChangeEvent & mce) try
   {
      zmq::message_t message(sizeof(GeneralMessage) + mce.GetBufferSize());
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      pMsgData->subscribeId = -1; // all trackers
      pMsgData->messageId = MessageId::MAP_CHANGE;
      mce.WriteBytes(pMsgData + 1);
      gSocketPub->send(message);
   }
   catch (zmq::error_t & e)
   {
      gOutPub.Print(string("HandleMapChanged error_t: ") + e.what());
   }
   catch (const std::exception & e)
   {
      gOutPub.Print(string("HandleMapChanged exception: ") + e.what());
   }
   catch (...)
   {
      gOutPub.Print("HandleMapChanged: an exception was not caught");
   }

   virtual void HandlePauseRequested(bool b) try
   {
      zmq::message_t message(sizeof(GeneralMessage) + sizeof(bool));
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      pMsgData->subscribeId = -1; // all trackers
      pMsgData->messageId = MessageId::PAUSE_REQUESTED;
      bool * pBool = (bool *)(pMsgData + 1);
      *pBool = b;
      gSocketPub->send(message);
   }
   catch (zmq::error_t & e)
   {
      gOutPub.Print(string("HandlePauseRequested error_t: ") + e.what());
   }
   catch (const std::exception & e)
   {
      gOutPub.Print(string("HandlePauseRequested exception: ") + e.what());
   }
   catch (...)
   {
      gOutPub.Print("HandlePauseRequested: an exception was not caught");
   }

   virtual void HandleAcceptKeyFrames(bool b) try
   {
      zmq::message_t message(sizeof(GeneralMessage) + sizeof(bool));
      GeneralMessage * pMsgData = message.data<GeneralMessage>();
      pMsgData->subscribeId = -1; // all trackers
      pMsgData->messageId = MessageId::ACCEPT_KEYFRAMES;
      bool * pBool = (bool *)(pMsgData + 1);
      *pBool = b;
      gSocketPub->send(message);
   }
   catch (zmq::error_t & e)
   {
      gOutPub.Print(string("HandleAcceptKeyFrames error_t: ") + e.what());
   }
   catch (const std::exception & e)
   {
      gOutPub.Print(string("HandleAcceptKeyFrames exception: ") + e.what());
   }
   catch (...)
   {
      gOutPub.Print("HandleAcceptKeyFrames: an exception was not caught");
   }
};

PrivateMapperObserver gServerObserver;

int main(int argc, char * argv[]) try
{
   ParseParams(argc, argv);

   cv::FileStorage mapperFile(gMapperFilename, cv::FileStorage::READ);
   Settings settings;
   VerifySettings(mapperFile, gMapperFilename, settings);

   // Output welcome message
   stringstream ss1;
   ss1 << endl;
   ss1 << "ORB-SLAM2-TEAM Server" << endl;
   ss1 << "Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza" << endl;
   ss1 << "Copyright (C) 2018 Joe Bedard" << endl;
   ss1 << "This program comes with ABSOLUTELY NO WARRANTY;" << endl;
   ss1 << "This is free software, and you are welcome to redistribute it" << endl;
   ss1 << "under certain conditions. See LICENSE.txt." << endl << endl;
   ss1 << "Server.Address=" << settings.serverAddress << endl;
   ss1 << "Publisher.Address=" << settings.publisherAddress << endl;
   gOutMain.Print(NULL, ss1);

   zmq::context_t context(2);

   zmq::socket_t socketRep(context, ZMQ_REP);
   socketRep.setsockopt(ZMQ_RCVTIMEO, &settings.serverTimeout, sizeof(Settings::serverTimeout));
   socketRep.setsockopt(ZMQ_LINGER, &settings.serverLinger, sizeof(Settings::serverLinger));
   socketRep.bind(settings.serverAddress);
   ServerParam param;
   param.socket = &socketRep;

   zmq::socket_t socketPub(context, ZMQ_PUB);
   socketPub.setsockopt(ZMQ_RCVTIMEO, &settings.publisherTimeout, sizeof(Settings::publisherTimeout));
   socketPub.setsockopt(ZMQ_LINGER, &settings.publisherLinger, sizeof(Settings::publisherLinger));
   socketPub.bind(settings.publisherAddress);
   gSocketPub = &socketPub;

   //Load ORB Vocabulary
   ORBVocabulary vocab;
   SyncPrint::Print(NULL, "Loading ORB Vocabulary. This could take a while...");
   bool bVocLoad = vocab.loadFromTextFile(gVocabFilename);
   if (!bVocLoad)
   {
      SyncPrint::Print("Failed to open vocabulary file at: ", gVocabFilename);
      exit(-1);
   }
   SyncPrint::Print(NULL, "Vocabulary loaded!");

   MapperServer mapperServer(vocab, false);
   mapperServer.AddObserver(&gServerObserver);
   gMapper = &mapperServer;
   thread serverThread(RunServer, &param);
   MapDrawer mapDrawer(mapperFile, mapperServer);

   //Initialize and start the Viewer thread
   Viewer viewer(NULL, &mapDrawer, NULL, mapperServer);
   viewer.Run(); //ends when window is closed
   gShouldRun = false; //signal server threads to stop

   /*
   gOutMain.Print("Press X to exit.");
   int key = 0;
   while (key != 'x' && key != 'X' && key != 27) // 27 == Esc
   {
      sleep(1000);
      key = _getch();
   }
   gShouldRun = false; //signal threads to stop
   */

   gOutMain.Print(NULL, "Shutting down server...");
   serverThread.join();

   return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
   gOutMain.Print(string("exception: ") + e.what());
   return EXIT_FAILURE;
}
catch (...)
{
   gOutMain.Print("an exception was not caught in main");
   return EXIT_FAILURE;
}
