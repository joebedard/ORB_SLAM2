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
#include <mutex>
#include "simple_msgs\point.h"
#include "simple\client.hpp"
#include "simple\subscriber.hpp"
#include "Sleep.h"

/****************************************
basic program for testing simple/zeromq
****************************************/

std::mutex mutexOutput;

void subscriber_callback(const simple_msgs::Point& p)
{
   std::unique_lock<std::mutex> lock(mutexOutput);
   std::cout << "Received point: " << p << std::endl;
}

int main(int argc, char * argv[]) try
{
   simple::Subscriber<simple_msgs::Point> subscriber{ "tcp://127.0.0.1:6000", subscriber_callback };
   std::cout << "Press a key to send 10 points.\n";
   std::cin.get();

   simple::Client<simple_msgs::Point> client{ "tcp://127.0.0.1:5000" };

   simple_msgs::Point p{ 0.0, 6.0, 7.0 };
   for (auto i = 0; i < 10; ++i)
   {
      mutexOutput.lock();
      p.setX((double)i);
      std::cout << "Sending point: " << p;
      if (client.request(p))
      {
         std::cout << "... request succeeded \n" << std::endl;
      }
      else
      {
         std::cerr << "... request failed" << std::endl;
      }
      mutexOutput.unlock();
      sleep(1000000);
   }

   std::cout << "Press a key to exit.\n";
   std::cin.get();

   return 0;
}
catch (const std::exception& e)
{
   std::cerr << e.what() << std::endl;
   return -1;
}
catch (...)
{
   std::cerr << std::endl << "An exception was not caught in the main thread." << std::endl;
   return EXIT_FAILURE;
}
