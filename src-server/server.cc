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

// reply server variables
struct ServerParam
{
   int returnCode;
   std::string serverAddress;
   int timeout;
   int linger;
   std::string publisherAddress;
};
bool gShouldRun = true;

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
   ReplyCode * pReplyCode = (ReplyCode *)reply.data();
   *pReplyCode = code;
   char * pStr = (char *)(pReplyCode + 1);
   strcpy(pStr, str);
   return reply;
}

zmq::message_t HelloService(void * requestData, size_t requestSize)
{
   gOutServ.Print(string("Received ") + (const char *)requestData);
   if (0 == strcmp((const char *)requestData, "Hello"))
   {
      gOutServ.Print("Replying World");
      return BuildReplyString(ReplyCode::Succeeded, "World");
   }
   else
   {
      gOutServ.Print("Replying Hello");
      return BuildReplyString(ReplyCode::Succeeded, "Hello");
   }
}

zmq::message_t LoginService(void * requestData, size_t requestSize)
{
   zmq::message_t reply;
   return reply;
}

zmq::message_t LogoutService(void * requestData, size_t requestSize)
{
   zmq::message_t reply;
   return reply;
}

// array of function pointer
zmq::message_t(*gServices[ServiceID::QUANTITY]) (void * requestData, size_t requestSize) = { HelloService, LoginService, LogoutService };

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
            ServiceID * pServiceID = (ServiceID *)request.data();
            if (*pServiceID < ServiceID::QUANTITY)
            {
               zmq::message_t reply = gServices[*pServiceID](pServiceID + 1, request.size());
               socket.send(reply);
            }
            else
            {
               zmq::message_t reply = BuildReplyString(ReplyCode::UnknownService, "Unknown Service");
               socket.send(reply);
            }
         } 
         catch (std::exception & e)
         {
            zmq::message_t reply = BuildReplyString(ReplyCode::Failed, e.what());
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

   thread serverThread(RunServer, &param);

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
   ss1 << "Press X to exit." << endl << endl;
   gOutMain.Print(NULL, ss1);

   int key = 0;
   while (key != 'x' && key != 'X' && key != 27) // 27 == Esc
   {
      sleep(1000);
      key = _getch();
   }
   gOutMain.Print(NULL, "Shutting down server...");
   gShouldRun = false; //signal threads to stop
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
