/**
* This file is part of ORB-SLAM2.
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SLEEP_H
#define SLEEP_H

#ifdef LINUX
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

inline void sleep(int microseconds)
{
#ifdef LINUX
	usleep(microseconds);   // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef _WIN32
	Sleep(microseconds / 1000);
#endif
}

#endif // SLEEP_H