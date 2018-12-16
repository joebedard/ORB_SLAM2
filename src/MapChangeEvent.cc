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
   MapChangeEvent::MapChangeEvent() : SyncPrint("MapChangeEvent: ")
   {

   }

   size_t MapChangeEvent::GetBufferSize()
   {
      size_t msgSize = 0;

      msgSize += KeyFrame::GetSetBufferSize(updatedKeyFrames);

      msgSize += MapPoint::GetSetBufferSize(updatedMapPoints);

      //msgSize += Serializer::GetVectorBufferSize<MapPointReplacement>(replacedMapPoints.size());

      //msgSize += Serializer::GetSetBufferSize<id_type>(deletedMapPoints.size());

      msgSize += Serializer::GetSetBufferSize<id_type>(deletedKeyFrames.size());

      return msgSize;
   }

   void * MapChangeEvent::ReadBytes(void * const buffer, Map & map)
   {
      Print("begin ReadBytes");
      std::unordered_map<id_type, MapPoint *> newMapPoints;
      std::unordered_map<id_type, KeyFrame *> newKeyFrames;

      void * pData = buffer;

      //Print("1");
      pData = KeyFrame::ReadSet(pData, map, newKeyFrames, newMapPoints, updatedKeyFrames);
      stringstream ss1; ss1 << "updated " << updatedKeyFrames.size() << " KeyFrames: ";
      for (KeyFrame * it : updatedKeyFrames)
      {
         ss1 << " " << it->GetId();
      }
      Print(ss1);
      stringstream ss2; ss2 << "created " << newMapPoints.size() << " MapPoints: ";
      for (pair<id_type, MapPoint *> it : newMapPoints)
      {
         ss2 << " " << it.first;
      }
      Print(ss2);

      //Print("2");
      pData = MapPoint::ReadSet(pData, map, newKeyFrames, newMapPoints, updatedMapPoints);
      stringstream ss3; ss3 << "updated " << updatedMapPoints.size() << " MapPoints: ";
      for (MapPoint * it : updatedMapPoints)
      {
         ss3 << " " << it->GetId();
      }
      Print(ss3);
      if (newMapPoints.size() != updatedMapPoints.size())
      {
         stringstream ss;
         ss << "MapPoint::ReadVector newMapPoints.size() != mapPoints.size() " << newMapPoints.size() << "!=" << updatedMapPoints.size();
         Print(ss);
         throw exception(ss.str().c_str());
      }

      //Print("3");
      // rebuild KeyFrame connections - they are not serialized to save bandwidth
      for (KeyFrame * pKF : updatedKeyFrames)
      {
         pKF->UpdateConnections();
      }

      //pData = ReadReplacements(pData, map, newMapPoints, replacedMapPoints);

      //pData = Serializer::ReadSet<id_type>(pData, deletedMapPoints);

      //Print("4");
      pData = Serializer::ReadSet<id_type>(pData, deletedKeyFrames);
      stringstream ss4; ss4 << "deleted " << deletedKeyFrames.size() << " KeyFrames: ";
      for (id_type it : deletedKeyFrames)
      {
         ss4 << " " << it;
      }
      Print(ss4);

      Print("end ReadBytes");
      return pData;
   }

   void * MapChangeEvent::WriteBytes(void * const buffer)
   {
      //Print("begin WriteBytes");
      void * pData = (void *)buffer;

      //Print("1");
      pData = KeyFrame::WriteSet(pData, updatedKeyFrames);

      //Print("3");
      pData = MapPoint::WriteSet(pData, updatedMapPoints);

      //pData = WriteReplacements(pData, replacedMapPoints);

      //Print("2");
      //pData = Serializer::WriteSet<id_type>(pData, deletedMapPoints);

      //Print("4");
      pData = Serializer::WriteSet<id_type>(pData, deletedKeyFrames);

      //Print("end WriteBytes");
      return pData;
   }

   void * MapChangeEvent::ReadReplacements(
      void * const buffer,
      Map & map,
      std::unordered_map<id_type, MapPoint *> & newMapPoints,
      std::unordered_map<id_type, MapPoint *> & replacements)
   {
      replacements.clear();
      size_t * pQuantity = (size_t *)buffer;
      MapPointReplacement * pData = (MapPointReplacement *)(pQuantity + 1);
      MapPointReplacement * pEnd = pData + *pQuantity;
      while (pData < pEnd)
      {
         MapPoint * pMP = MapPoint::Find(pData->replacerId, map, newMapPoints);
         if (pMP == NULL)
         {
            std::stringstream ss;
            ss << "MapChangeEvent::ReadReplacements detected an unknown MapPoint with id=" << pData->replacerId;
            throw exception(ss.str().c_str());
         }
         replacements[pData->deletedId] = pMP;
         ++pData;
      }
      return pData;
   }

   void * MapChangeEvent::WriteReplacements(void * const buffer, std::unordered_map<id_type, MapPoint *> & replacements)
   {
      size_t * pQuantity = (size_t *)buffer;
      *pQuantity = replacements.size();
      MapPointReplacement * pData = (MapPointReplacement *)(pQuantity + 1);
      for (std::pair<id_type, MapPoint *> p : replacements)
      {
         pData->deletedId = p.first;
         pData->replacerId = p.second->GetId();
         ++pData;
      }
      return pData;
   }

}