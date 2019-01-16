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

#ifndef MAPPERSUBJECT_H
#define MAPPERSUBJECT_H

#include <set>
#include <mutex>
#include "MapperObserver.h"

namespace ORB_SLAM2_TEAM
{

   /*
       base class for Subject class from the Observer Pattern
       thread-safe methods encapsulate a collection of observers
       see https://sourcemaking.com/design_patterns/observer
   */
   class MapperSubject
   {
   public:

      MapperSubject()
      {

      }

      void AddObserver(MapperObserver * ob)
      {
         if (!ob) return;
         std::unique_lock<std::mutex> lock(mMutex);
         mObservers.insert(ob);
      }

      void RemoveObserver(MapperObserver * ob)
      {
         if (!ob) return;
         std::unique_lock<std::mutex> lock(mMutex);
         mObservers.erase(ob);
      }

   protected:

      void NotifyMapReset()
      {
         unique_lock<mutex> lock(mMutex);

         for (auto it : mObservers)
         {
            it->HandleMapReset();
         }
      }

      void NotifyMapChanged(MapChangeEvent & mce)
      {
         if (mce.empty()) return;
         unique_lock<mutex> lock(mMutex);

         for (auto it : mObservers)
         {
            it->HandleMapChanged(mce);
         }
      }

      void NotifyMapChanged(Map & theMap)
      {
         MapChangeEvent mapChanges;

         set<KeyFrame *> setKFs = theMap.GetKeyFrameSet();
         for (KeyFrame * pKF : setKFs)
         {
            if (pKF->GetModified())
            {
               if (pKF->IsBad())
                  mapChanges.deletedKeyFrames.insert(pKF->id);
               else
                  mapChanges.updatedKeyFrames.insert(pKF);
            }
         }

         set<MapPoint *> setMPs = theMap.GetMapPointSet();
         for (MapPoint * pMP : setMPs)
         {
            if (pMP->GetModified())
            {
               if (pMP->IsBad())
                  mapChanges.deletedMapPoints.insert(pMP->id);
               else
                  mapChanges.updatedMapPoints.insert(pMP);
            }
         }

         NotifyMapChanged(mapChanges);

         for (KeyFrame * pKF : setKFs)
         {
            if (pKF->GetModified())
            {
               pKF->SetModified(false);
            }
         }

         for (MapPoint * pMP : setMPs)
         {
            if (pMP->GetModified())
            {
               pMP->SetModified(false);
            }
         }

      }

      void NotifyPauseRequested(bool b)
      {
         std::unique_lock<std::mutex> lock(mMutex);

         for (auto it : mObservers)
         {
            it->HandlePauseRequested(b);
         }
      }

      void NotifyIdle(bool b)
      {
         std::unique_lock<std::mutex> lock(mMutex);

         for (auto it : mObservers)
         {
            it->HandleIdle(b);
         }
      }

   private:

      std::mutex mMutex;

      std::set<MapperObserver *> mObservers;

   };

}

#endif // MAPPERSUBJECT_H