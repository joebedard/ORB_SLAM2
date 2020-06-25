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

#ifndef DURATION_H
#define DURATION_H

#include <chrono>

typedef std::chrono::steady_clock::time_point time_type;
inline std::chrono::steady_clock::time_point GetNow()
{
   return std::chrono::steady_clock::now();
}

inline double Duration(time_type ending, time_type starting)
{
   return std::chrono::duration_cast<std::chrono::duration<double>>(ending - starting).count();  
}

#endif // DURATION_H