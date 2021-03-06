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

#ifndef MAPOBSERVER_H
#define MAPOBSERVER_H

#include "MapChangeEvent.h"

namespace ORB_SLAM2_TEAM
{

   /*
       base class for Observer class from the Observer Pattern
       see https://sourcemaking.com/design_patterns/observer
   */
   class MapObserver
   {
   public:

      virtual void HandleMapReset() {}

      virtual void HandleMapChanged(MapChangeEvent & mce) {}

   };

}

#endif // MAPOBSERVER_H