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

#include "Mapper.h"
#include "Optimizer.h"
#include "Sleep.h"
#include <exception>

namespace ORB_SLAM2
{

   Mapper::Mapper(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular)
      : mpMap(pMap), mpVocab(pVocab), mbMonocular(bMonocular), mInitialized(false)
   {
      if (pMap == NULL)
         throw std::exception("pMap must not be NULL");

      if (pVocab == NULL)
         throw std::exception("pVocab must not be NULL");

      assert(mTrackers.size() == MAX_TRACKERS + 1);

      mpKeyFrameDB = new KeyFrameDatabase(*pVocab);

      for (int i = 0; i < MAX_TRACKERS + 1; ++i)
      {
         mTrackers[i].connected = false;
         mTrackers[i].lastKeyFrameId = 0;
         mTrackers[i].lastMapPointId = 0;
      }
   }

   std::mutex & Mapper::getMutexMapUpdate()
   {
      return mpMap->mMutexMapUpdate;
   }

   long unsigned Mapper::KeyFramesInMap()
   {
      return mpMap->KeyFramesInMap();
   }

   void Mapper::Reset()
   {
      // Reset Local Mapping
      cout << "Reseting Local Mapper...";
      mpLocalMapper->RequestReset();
      cout << " done" << endl;

      // Reset Loop Closing
      cout << "Reseting Loop Closing...";
      mpLoopCloser->RequestReset();
      cout << " done" << endl;

      // Clear BoW Database
      cout << "Reseting Database...";
      mpKeyFrameDB->clear();
      cout << " done" << endl;

      // Clear Map (this erase MapPoints and KeyFrames)
      mpMap->clear();

      Frame::nNextId = 0;
      mInitialized = false;
   }

   std::vector<KeyFrame*> Mapper::DetectRelocalizationCandidates(Frame* F)
   {
      return mpKeyFrameDB->DetectRelocalizationCandidates(F);
   }
      
   bool Mapper::GetInitialized()
   {
      return mInitialized;
   }

   bool Mapper::GetPauseRequested()
   {
      return mpLocalMapper->PauseRequested();
   }
   
   bool Mapper::AcceptKeyFrames()
   {
      return mpLocalMapper->AcceptKeyFrames();
   }
   
   void Mapper::Shutdown()
   {
      mpLocalMapper->RequestFinish();
      mpLoopCloser->RequestFinish();

      // Wait until all thread have effectively stopped
      while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
      {
         sleep(5000);
      }
   }

   void Mapper::Initialize(unsigned int trackerId, Map & pMap)
   {
      if (mInitialized)
         throw std::exception("The mapper may only be initialized once.");

      if (trackerId != 1)
         throw std::exception("Only the first Tracker may initialize the map.");

      //Initialize the Local Mapping thread and launch
      mpLocalMapper = new LocalMapping(mpMap, mpKeyFrameDB, mbMonocular, 0, MAX_TRACKERS + 1);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

      //Initialize the Loop Closing thread and launch
      mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDB, mpVocab, !mbMonocular);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

      mpLocalMapper->SetLoopCloser(mpLoopCloser);
      mpLoopCloser->SetLocalMapper(mpLocalMapper);

      auto allKFs = pMap.GetAllKeyFrames();
      for (auto it = allKFs.begin(); it != allKFs.end(); it++)
      {
         InsertKeyFrame(trackerId, *it);
      }

      mInitialized = true;
   }

   bool Mapper::InsertKeyFrame(unsigned int trackerId, KeyFrame *pKF)
   {
      if (mpLocalMapper->InsertKeyFrame(pKF))
      {
         // update TrackerStatus array with last Id(s)

         assert((pKF->GetId() - trackerId) % KEYFRAME_ID_SPAN == 0);
         if (mTrackers[trackerId].lastKeyFrameId < pKF->GetId())
         {
            mTrackers[trackerId].lastKeyFrameId = pKF->GetId();
         }

         set<ORB_SLAM2::MapPoint*> mapPoints = pKF->GetMapPoints();
         for (auto pMP : mapPoints)
         {
            assert((pMP->GetId() - trackerId) % MAPPOINT_ID_SPAN == 0);
            if (mTrackers[trackerId].lastMapPointId < pMP->GetId())
            {
               mTrackers[trackerId].lastMapPointId = pMP->GetId();
            }
         }

         return true;
      }
      else
         return false;
   }

   unsigned int Mapper::LoginTracker(unsigned long  & firstKeyFrameId, unsigned int & keyFrameIdSpan, unsigned long & firstMapPointId, unsigned int & mapPointIdSpan)
   {
      unsigned int id;
      /*
      Note that mTrackers[0] is not used. It might be used in the future by the LocalMapper.
      */
      for (id = 1; id <= MAX_TRACKERS; ++id)
      {
         if (!mTrackers[id].connected)
         {
            mTrackers[id].connected = true;
            break;
         }
      }

      if (id > MAX_TRACKERS)
         throw std::exception("Maximum number of trackers reached. Additional trackers are not supported.");

      if (0 == mTrackers[id].lastKeyFrameId)
         firstKeyFrameId = id;
      else
         firstKeyFrameId = mTrackers[id].lastKeyFrameId + KEYFRAME_ID_SPAN;

      keyFrameIdSpan = KEYFRAME_ID_SPAN;

      if (0 == mTrackers[id].lastMapPointId)
         firstMapPointId = id;
      else
         firstMapPointId = mTrackers[id].lastMapPointId + MAPPOINT_ID_SPAN;

      mapPointIdSpan = MAPPOINT_ID_SPAN;

      return id;
   }

   void Mapper::LogoutTracker(unsigned int id)
   {
      mTrackers[id].connected = false;
   }
}