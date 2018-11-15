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

#include "MapChangeEvent.h"
#include "Serializer.h"

namespace ORB_SLAM2
{
   MapChangeEvent::MapChangeEvent()
   {

   }

   size_t MapChangeEvent::GetBufferSize()
   {
      size_t msgSize = 0;

      msgSize += KeyFrame::GetSetBufferSize(updatedKeyFrames);

      msgSize += Serializer::GetSetBufferSize<id_type>(deletedKeyFrames.size());

      msgSize += MapPoint::GetSetBufferSize(updatedMapPoints);

      msgSize += Serializer::GetSetBufferSize<id_type>(deletedMapPoints.size());

      return msgSize;
   }

   void * MapChangeEvent::ReadBytes(const void * buffer, Map & map)
   {
      std::unordered_map<id_type, KeyFrame *> newKeyFrames;
      std::unordered_map<id_type, MapPoint *> newMapPoints;

      char * pData = (char *)buffer;

      pData = (char *)KeyFrame::ReadSet(pData, map, newKeyFrames, newMapPoints, updatedKeyFrames);

      pData = (char *)Serializer::ReadSet<id_type>(pData, deletedKeyFrames);

      pData = (char *)MapPoint::ReadSet(pData, map, newKeyFrames, newMapPoints, updatedMapPoints);

      pData = (char *)Serializer::ReadSet<id_type>(pData, deletedMapPoints);

      return pData;
   }

   void * MapChangeEvent::WriteBytes(const void * buffer)
   {
      char * pData = (char *)buffer;

      pData = (char *)KeyFrame::WriteSet(pData, updatedKeyFrames);

      pData = (char *)Serializer::WriteSet<id_type>(pData, deletedKeyFrames);

      pData = (char *)MapPoint::WriteSet(pData, updatedMapPoints);

      pData = (char *)Serializer::WriteSet<id_type>(pData, deletedMapPoints);

      return pData;
   }

}