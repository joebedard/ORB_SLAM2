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

#include <iostream>
#include "simple_msgs\point.h"
#include "simple\server.hpp"
#include "simple\publisher.hpp"

/****************************************
 basic program for testing simple/zeromq
****************************************/

simple::Publisher<simple_msgs::Point> * pPublisher = NULL;

void server_callback(simple_msgs::Point& p)
{
   std::cout << "Received a point: " << p << std::endl;
   pPublisher->publish(p);
}

int main(int argc, char * argv[]) try
{
   simple::Publisher<simple_msgs::Point> publisher{ "tcp://*:6000" };
   pPublisher = &publisher;
   simple::Server<simple_msgs::Point> server{ "tcp://*:5000", server_callback };

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