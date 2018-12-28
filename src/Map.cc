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
      if (!pKF)
         throw exception("Map::AddKeyFrame: can not add a NULL KeyFrame *");

      unique_lock<mutex> lock(mMutexMap);
      mKeyFrames[pKF->id] = pKF;
      if (pKF->id > mnMaxKFid)
         mnMaxKFid = pKF->id;
      Print("end AddKeyFrame");
   }

   void Map::AddMapPoint(MapPoint *pMP)
   {
      if (!pMP)
         throw exception("Map::AddMapPoint: can not add a NULL MapPoint *");

      unique_lock<mutex> lock(mMutexMap);
      mMapPoints[pMP->id] = pMP;
   }

   void Map::EraseMapPoint(MapPoint *pMP)
   {
      unique_lock<mutex> lock(mMutexMap);
      mMapPoints.erase(pMP->id);

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
      mKeyFrames.erase(pKF->id);

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

   set<KeyFrame *> Map::GetKeyFrameSet()
   {
      Print("begin GetKeyFrameSet");
      unique_lock<mutex> lock(mMutexMap);
      set<KeyFrame *> allKeyFrames;
      for (unordered_map<id_type, KeyFrame *>::iterator it = mKeyFrames.begin(); it != mKeyFrames.end(); ++it)
      {
         if (it->second == NULL) Print("it->second == NULL");
         allKeyFrames.insert(it->second);
      }
      Print("end GetKeyFrameSet");
      return allKeyFrames;
   }

   KeyFrame * Map::GetKeyFrame(const id_type keyFrameId) const
   {
      //unique_lock<mutex> lock(mMutexMap);
      return (mKeyFrames.count(keyFrameId) == 1) ? mKeyFrames.at(keyFrameId) : NULL;
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

   set<MapPoint *> Map::GetMapPointSet()
   {
      Print("begin GetMapPointSet");
      unique_lock<mutex> lock(mMutexMap);
      set<MapPoint *> allMapPoints;
      for (unordered_map<id_type, MapPoint *>::iterator it = mMapPoints.begin(); it != mMapPoints.end(); ++it)
      {
         if (it->second == NULL) Print("it->second == NULL");
         allMapPoints.insert(it->second);
      }
      Print("end GetMapPointSet");
      return allMapPoints;
   }

   MapPoint * Map::GetMapPoint(id_type mapPointId) const
   {
      //unique_lock<mutex> lock(mMutexMap);
      return (mMapPoints.count(mapPointId) == 1) ? mMapPoints.at(mapPointId) : NULL;
   }

   size_t Map::MapPointsInMap()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mMapPoints.size();
   }

   size_t Map::KeyFramesInMap()
   {
      unique_lock<mutex> lock(mMutexMap);
      return mKeyFrames.size();
   }

   id_type Map::GetMaxKFid()
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

   void Map::Link(MapPoint & rMP, size_t idx, KeyFrame & rKF) 
   {
      Print("begin Link");
      unique_lock<mutex> lock1(mMutexMap);
      unique_lock<mutex> lock2(rMP.mMutexFeatures);
      unique_lock<mutex> lock3(rKF.mMutexFeatures);

      MapPoint * prevMP = rKF.mvpMapPoints.at(idx);
      if (prevMP != NULL) {
         if (prevMP == &rMP) {
            // the MP and KF are already linked
            return;
         }
         else {
            // rKF is already linked to a different MP, so unlink them
            rKF.mvpMapPoints.at(idx) = NULL;
            unique_lock<mutex> lock(prevMP->mMutexFeatures);
            prevMP->mObservations.erase(&rKF);
            prevMP->CompleteUnlink(idx, rKF);
         }
      }

      if (rMP.mObservations.count(&rKF) > 0) {
         size_t prevIdx = rMP.mObservations[&rKF];
         if (prevIdx != idx) {
            // rMP is already linked to rKF with a different index, so unlink them
            rKF.mvpMapPoints.at(prevIdx) = NULL;
            rMP.mObservations.erase(&rKF);
            rMP.CompleteUnlink(prevIdx, rKF);
         }
      }

      // link rMP to rKF at the desired index
      rKF.mvpMapPoints.at(idx) = &rMP;
      rMP.mObservations[&rKF] = idx;
      rMP.CompleteLink(idx, rKF);
      rKF.SetModified(true);
      Print("end Link");
   }

   void Map::Unlink(MapPoint & rMP, KeyFrame & rKF) 
   {
      Print("begin Unlink");
      unique_lock<mutex> lock1(mMutexMap);
      unique_lock<mutex> lock2(rMP.mMutexFeatures);
      unique_lock<mutex> lock3(rKF.mMutexFeatures);

      if (rMP.mObservations.count(&rKF) > 0) {
         size_t prevIdx = rMP.mObservations[&rKF];
         rKF.mvpMapPoints.at(prevIdx) = NULL;
         rMP.mObservations.erase(&rKF);

         rKF.SetModified(true);
         rMP.CompleteUnlink(prevIdx, rKF);
      }
      Print("end Unlink");
   }

   void Map::Replace(MapPoint & oldMP, MapPoint & newMP) {
      Print("begin Replace");
      if (oldMP.id == newMP.id)
      {
         Print("end Replace 1");
         return;
      }
      map<KeyFrame *, size_t> obs;
      {
         unique_lock<mutex> lock1(mMutexMap);
         unique_lock<mutex> lock2(oldMP.mMutexFeatures);
         obs = oldMP.mObservations;
      }
      for (pair<KeyFrame *, size_t> p : obs) {
         KeyFrame & rKF = *p.first;
         Link(newMP, p.second, rKF);
      }
      Print("end Replace 2");
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
