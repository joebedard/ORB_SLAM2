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


#ifndef ENUMS_H
#define ENUMS_H

namespace ORB_SLAM2_TEAM
{

   // Tracking states
   enum TrackingState {
      NO_IMAGES_YET = 0,
      NOT_INITIALIZED = 1,
      TRACKING_OK = 2,
      TRACKING_LOST = 3
   };

   // Input sensor
   enum SensorType {
      MONOCULAR = 0,
      STEREO = 1,
      RGBD = 2
   };

   enum ReplyCode
   {
      UNKNOWN_SERVICE = 0,
      SUCCEEDED = 1,
      FAILED = 2
   };

   enum ServiceId
   {
      HELLO = 0,
      LOGIN_TRACKER = 1,
      LOGOUT_TRACKER = 2,
      INITIALIZE_MONO = 3,
      INITIALIZE_STEREO = 4,
      GET_MAP = 5,
      UPDATE_POSE = 6,
      INSERT_KEYFRAME = 7,
      RESET = 8,
      quantityServiceId = 9
   };

   enum MessageId
   {
      MAP_RESET = 0,
      MAP_CHANGE = 1,
      PAUSE_REQUESTED = 2,
      ACCEPT_KEYFRAMES = 3,
      PIVOT_UPDATE = 4,
      POSE_UPDATE = 5,
      quantityMessageId = 6
   };

}

#endif // ENUMS_H