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


#ifndef TRACKINGSTATE_H
#define TRACKINGSTATE_H

namespace ORB_SLAM2
{

   // Tracking states
   // TODO: refactor out mapper states
   enum eTrackingState {
      SYSTEM_NOT_READY = -1,
      NO_IMAGES_YET = 0,
      NOT_INITIALIZED = 1,
      OK = 2,
      LOST = 3
   };

    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

}

#endif // TRACKINGSTATE_H