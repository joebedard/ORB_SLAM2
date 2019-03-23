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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "SyncPrint.h"
#include <set>
#include <unordered_map>

#include <mutex>

namespace ORB_SLAM2_TEAM
{
   class MapPoint;
   class KeyFrame;

   class Map : SyncPrint
   {
   public:
      
      mutex mutexMapUpdate;

      Map();

      void AddKeyFrame(KeyFrame * pKF);
      
      KeyFrame * GetFirstKeyFrame();

      void AddMapPoint(MapPoint * pMP);

      void EraseMapPoint(MapPoint * pMP);

      void EraseMapPoint(id_type mapPointId);

      void EraseKeyFrame(KeyFrame * pKF);

      void EraseKeyFrame(id_type keyFrameId);

      void InformNewBigChange();

      int GetLastBigChangeIdx();

      vector<KeyFrame *> GetAllKeyFrames();

      set<KeyFrame *> GetKeyFrameSet();

      KeyFrame * GetKeyFrame(id_type keyFrameId) const;

      vector<MapPoint *> GetAllMapPoints();

      set<MapPoint *> GetMapPointSet();

      MapPoint * GetMapPoint(id_type mapPointId) const;

      size_t MapPointsInMap();

      size_t  KeyFramesInMap();

      id_type GetMaxKFid();

      void Clear();

      void Link(KeyFrame & rKF, vector<MapPoint *> & mapPoints);

      void Link(MapPoint & rMP, size_t idx, KeyFrame & rKF);

      void Unlink(MapPoint & rMP, KeyFrame & rKF);

      void Replace(MapPoint & oldMP, MapPoint & newMP);

      // only for debugging
      void ValidateAllLinks();

      Map & operator=(const Map & rMap);

      vector<KeyFrame *> mvpKeyFrameOrigins;

   protected:

      id_type mnMaxKFid;

      // Index related to a big change in the map (loop closure, global BA)
      int mnBigChangeIdx;

      mutex mMutexMap;

   private:
      // This avoids that two points (with same id) are created simultaneously in separate threads (id conflict)
      mutex mMutexPointCreation;

      unordered_map<id_type, MapPoint *> mMapPoints;

      unordered_map<id_type, KeyFrame *> mKeyFrames;

      // returns a MapPoint if it was replaced, otherwise returns NULL
      MapPoint * LinkWithoutLock(MapPoint & rMP, size_t idx, KeyFrame & rKF);

      KeyFrame * mFirstKeyFrame;
   };

} //namespace ORB_SLAM

#endif // MAP_H
