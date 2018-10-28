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

#ifndef MAPCHANGEEVENT_H
#define MAPCHANGEEVENT_H

#include <set>
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2
{

   // collects all map changes into a composite event
   class MapChangeEvent
   {
   public:

      set<KeyFrame *> updatedKeyFrames;

      set<unsigned long int> deletedKeyFrames;

      set<MapPoint *> updatedMapPoints;

      set<unsigned long int> deletedMapPoints;

      bool empty()
      {
         return updatedKeyFrames.empty() && deletedKeyFrames.empty() && updatedMapPoints.empty() && deletedMapPoints.empty();
      }
   };

}

#endif // MAPCHANGEEVENT_H