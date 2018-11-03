/**
* This file is part of ORB-SLAM2-TEAM.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

   Map::Map()
      : mnMaxKFid(0), mnBigChangeIdx(0), SyncPrint("Map: ", false)
   {

   }

   void Map::AddKeyFrame(KeyFrame *pKF)
   {
      Print("begin AddKeyFrame");
      unique_lock<mutex> lock(mMutexMap);
      mKeyFrames[pKF->GetId()] = pKF;
      if (pKF->GetId() > mnMaxKFid)
         mnMaxKFid = pKF->GetId();
      Print("end AddKeyFrame");
   }

   void Map::AddMapPoint(MapPoint *pMP)
   {
      unique_lock<mutex> lock(mMutexMap);
      mMapPoints[pMP->GetId()] = pMP;
   }

   void Map::EraseMapPoint(MapPoint *pMP)
   {
      unique_lock<mutex> lock(mMutexMap);
      mMapPoints.erase(pMP->GetId());

      // TODO: This only erase the pointer.
      // Delete the MapPoint
   }

   void Map::EraseMapPoint(id_type mapPointId)
   {
      unique_lock<mutex> lock(mMutexMap);
      mMapPoints.erase(mapPointId);

      // TODO: This only erase the pointer.
      // Delete the MapPoint
   }

   void Map::EraseKeyFrame(KeyFrame *pKF)
   {
      unique_lock<mutex> lock(mMutexMap);
      mKeyFrames.erase(pKF->GetId());

      // TODO: This only erase the pointer.
      // Delete the KeyFrame
   }

   void Map::EraseKeyFrame(id_type keyFrameId)
   {
      unique_lock<mutex> lock(mMutexMap);
      mKeyFrames.erase(keyFrameId);

      // TODO: This only erase the pointer.
      // Delete the KeyFrame
   }

   void Map::InformNewBigChange()
   {
      unique_lock<mutex> lock(mMutexMap);
      mnBigChangeIdx++;
   }

   int Map::GetLastBigChangeIdx()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mnBigChangeIdx;
   }

   vector<KeyFrame *> Map::GetAllKeyFrames()
   {
      Print("begin GetAllKeyFrames");
      unique_lock<mutex> lock(mMutexMap);
      vector<KeyFrame *> allKeyFrames;
      for (unordered_map<id_type, KeyFrame *>::iterator it = mKeyFrames.begin(); it != mKeyFrames.end(); ++it)
      {
         if (it->second == NULL) Print("it->second == NULL");
         allKeyFrames.push_back(it->second);
      }
      Print("end GetAllKeyFrames");
      return allKeyFrames;
   }

   KeyFrame * Map::GetKeyFrame(id_type keyFrameId)
   {
      unique_lock<mutex> lock(mMutexMap);
      return mKeyFrames[keyFrameId];
   }

   vector<MapPoint *> Map::GetAllMapPoints()
   {
      Print("begin GetAllMapPoints");
      unique_lock<mutex> lock(mMutexMap);
      vector<MapPoint *> allMapPoints;
      for (unordered_map<id_type, MapPoint *>::iterator it = mMapPoints.begin(); it != mMapPoints.end(); ++it)
      {
         if (it->second == NULL) Print("it->second == NULL");
         allMapPoints.push_back(it->second);
      }
      Print("end GetAllMapPoints");
      return allMapPoints;
   }

   MapPoint * Map::GetMapPoint(id_type mapPointId)
   {
      unique_lock<mutex> lock(mMutexMap);
      return mMapPoints[mapPointId];
   }

   long unsigned int Map::MapPointsInMap()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mMapPoints.size();
   }

   long unsigned int Map::KeyFramesInMap()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mKeyFrames.size();
   }

   long unsigned int Map::GetMaxKFid()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mnMaxKFid;
   }

   void Map::Clear()
   {
      unique_lock<mutex> lock(mMutexMap);

      for (pair<id_type, MapPoint *> p : mMapPoints)
         delete p.second;

      for (pair<id_type, KeyFrame *> p : mKeyFrames)
         delete p.second;

      mMapPoints.clear();
      mKeyFrames.clear();
      mnMaxKFid = 0;
      mvpKeyFrameOrigins.clear();
   }

   Map & Map::operator=(const Map & map)
   {
      unique_lock<mutex> lock(mMutexMap);

      if (this != _STD addressof(map))
      {
         this->mvpKeyFrameOrigins = map.mvpKeyFrameOrigins;
         this->mMapPoints = map.mMapPoints;
         this->mKeyFrames = map.mKeyFrames;
         this->mnMaxKFid = map.mnMaxKFid;
         this->mnBigChangeIdx = map.mnBigChangeIdx;
      }
      return (*this);
   }

} //namespace ORB_SLAM
