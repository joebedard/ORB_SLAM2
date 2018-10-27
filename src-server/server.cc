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

using namespace ORB_SLAM2;

/***
   ORB_SLAM2 Server with shared map for multiple tracking clients.
***/

// logging variables
SyncPrint gOutMain("main: ");
SyncPrint gOutServ("server: ");

// command line parameters
char * gVocabFilename = NULL;
char * gMapperFilename = NULL;

// server variables
struct ServerParam
{
   int returnCode;
   std::string serverAddress;
   int timeout;
   int linger;
   std::string publisherAddress;
};
bool gShouldRun = true;
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

void VerifySettings(cv::FileStorage & settings, const char * settingsFilePath, ServerParam & param)
{
   if (!settings.isOpened())
   {
      std::string m("Failed to open settings file at: ");
      m.append(settingsFilePath);
      throw std::exception(m.c_str());
   }

   param.serverAddress.append(settings["Server.Address"]);
   if (0 == param.serverAddress.length())
      throw std::exception("Server.Address property is not set or value is not in quotes.");

   param.publisherAddress.append(settings["Publisher.Address"]);
   if (0 == param.publisherAddress.length())
      throw std::exception("Publisher.Address property is not set or value is not in quotes.");

   param.timeout = settings["Server.Timeout"];

   param.linger = settings["Server.Linger"];

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
   GeneralRequest * pReqData = request.data<GeneralRequest>();
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

zmq::message_t LoginService(zmq::message_t & request)
{
   gOutServ.Print("begin LoginService");
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
   gMapper->LoginTracker(pivotCalib, pRepData->trackerId, pRepData->firstKeyFrameId, pRepData->keyFrameIdSpan,
      pRepData->firstMapPointId, pRepData->mapPointIdSpan);
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end LoginService");
   return reply;
}

zmq::message_t LogoutService(zmq::message_t & request)
{
   gOutServ.Print("begin LogoutService");
   LogoutTrackerRequest * pReqData = request.data<LogoutTrackerRequest>();
   pReqData->trackerId;

   zmq::message_t reply(sizeof(GeneralReply));
   GeneralReply * pRepData = reply.data<GeneralReply>();
   gMapper->LogoutTracker(pReqData->trackerId);
   pRepData->replyCode = ReplyCode::SUCCEEDED;

   gOutServ.Print("end LogoutService");
   return reply;
}

// array of function pointer
zmq::message_t (*gServices[ServiceId::quantity])(zmq::message_t & request) = { HelloService, LoginService, LogoutService };

void RunServer(void * param) try
{
   ServerParam * serverParam = (ServerParam *)param;

   zmq::context_t context(1);
   zmq::socket_t socket(context, ZMQ_REP);
   socket.setsockopt(ZMQ_RCVTIMEO, &serverParam->timeout, sizeof(ServerParam::timeout));
   socket.setsockopt(ZMQ_LINGER, &serverParam->linger, sizeof(ServerParam::linger));
   socket.bind(serverParam->serverAddress);

   zmq::message_t request;
   while (gShouldRun) 
   {
      if (socket.recv(&request, ZMQ_NOBLOCK))
      {
         try 
         {
            GeneralRequest * pReqData = request.data<GeneralRequest>();
            if (pReqData->serviceId < ServiceId::quantity)
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

int main(int argc, char * argv[]) try
{
   ParseParams(argc, argv);

   cv::FileStorage mapperSettings(gMapperFilename, cv::FileStorage::READ);
   ServerParam param;
   VerifySettings(mapperSettings, gMapperFilename, param);

   // Output welcome message
   stringstream ss1;
   ss1 << endl;
   ss1 << "ORB-SLAM2-NET Server" << endl;
   ss1 << "Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza" << endl;
   ss1 << "Copyright (C) 2018 Joe Bedard" << endl;
   ss1 << "This program comes with ABSOLUTELY NO WARRANTY;" << endl;
   ss1 << "This is free software, and you are welcome to redistribute it" << endl;
   ss1 << "under certain conditions. See LICENSE.txt." << endl << endl;
   ss1 << "Server.Address=" << param.serverAddress << endl;
   ss1 << "Publisher.Address=" << param.publisherAddress << endl;
   gOutMain.Print(NULL, ss1);

   ORBVocabulary vocab;
   MapperServer mapperServer(vocab, false);
   gMapper = &mapperServer;
   MapDrawer mapDrawer(mapperSettings, mapperServer);

   //Load ORB Vocabulary
   SyncPrint::Print(NULL, "Loading ORB Vocabulary. This could take a while...");
   bool bVocLoad = vocab.loadFromTextFile(gVocabFilename);
   if (!bVocLoad)
   {
      SyncPrint::Print("Failed to open vocabulary file at: ", gVocabFilename);
      exit(-1);
   }
   SyncPrint::Print(NULL, "Vocabulary loaded!");

   thread serverThread(RunServer, &param);

   //Initialize and start the Viewer thread
   Viewer viewer(NULL, &mapDrawer, NULL, &mapperServer);
   viewer.Run(); //ends when window is closed
   gShouldRun = false; //signal tracking threads to stop

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
