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
   long unsigned int Mapper::nNextMapPointId = 0;

   Mapper::Mapper(Map* pMap, ORBVocabulary* pVocab, const bool bMonocular)
      : mpMap(pMap), mState(NO_IMAGES_YET), mMutexMapUpdate(pMap->mMutexMapUpdate)
   {
      if (pMap == NULL)
         throw std::exception("pMap must not be NULL");

      //Create KeyFrame Database
      mpKeyFrameDB = new KeyFrameDatabase(*pVocab);

      //Initialize the Local Mapping thread and launch
      mpLocalMapper = new LocalMapping(mpMap, mpKeyFrameDB, bMonocular);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

      //Initialize the Loop Closing thread and launch
      mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDB, pVocab, !bMonocular);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

      mpLocalMapper->SetLoopCloser(mpLoopCloser);
      mpLoopCloser->SetLocalMapper(mpLocalMapper);
   }

   long unsigned Mapper::KeyFramesInMap()
   {
      return mpMap->KeyFramesInMap();
   }

   void Mapper::EraseKeyFrame(KeyFrame * pKF)
   {
      mpMap->EraseKeyFrame(pKF);
      mpKeyFrameDB->erase(pKF);
   }

   void Mapper::AddKeyFrame(KeyFrame * pKF)
   {
      mpMap->AddKeyFrame(pKF);
   }

   long unsigned int Mapper::NextMapPointId()
   {
      // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
      unique_lock<mutex> lock(mpMap->mMutexPointCreation);
      return nNextMapPointId++;
   }

   void Mapper::EraseMapPoint(MapPoint * pMP)
   {
      mpMap->EraseMapPoint(pMP);
   }

   void Mapper::AddMapPoint(MapPoint * pMP)
   {
      mpMap->AddMapPoint(pMP);
   }

   long unsigned int Mapper::MapPointsInMap()
   {
      return mpMap->MapPointsInMap();
   }

   std::vector<MapPoint*> Mapper::GetAllMapPoints()
   {
      return mpMap->GetAllMapPoints();
   }

   void Mapper::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs)
   {
      mpMap->SetReferenceMapPoints(vpMPs);
   }

   void Mapper::AddOriginKeyFrame(KeyFrame * pKF)
   {
      mpMap->mvpKeyFrameOrigins.push_back(pKF);
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

      KeyFrame::nNextId = 0;
      Frame::nNextId = 0;
      mState = NO_IMAGES_YET;
   }

   std::vector<KeyFrame*> Mapper::DetectRelocalizationCandidates(Frame* F)
   {
      return mpKeyFrameDB->DetectRelocalizationCandidates(F);
   }
   
   void Mapper::SetState(eTrackingState state)
   {
      mState = state;
   }
   
   eTrackingState Mapper::GetState()
   {
      return mState;
   }
   
   void Mapper::RequestStop()
   {
      mpLocalMapper->RequestStop();
   }
   
   bool Mapper::isStopped()
   {
      return mpLocalMapper->isStopped();
   }
   
   void Mapper::Release()
   {
      mpLocalMapper->Release();
   }
   
   bool Mapper::stopRequested()
   {
      return mpLocalMapper->stopRequested();
   }
   
   bool Mapper::AcceptKeyFrames()
   {
      return mpLocalMapper->AcceptKeyFrames();
   }
   
   void Mapper::InterruptBA()
   {
      mpLocalMapper->InterruptBA();
   }
   
   int Mapper::KeyframesInQueue()
   {
      return mpLocalMapper->KeyframesInQueue();
   }

   bool Mapper::SetNotStop(bool b)
   {
      return mpLocalMapper->SetNotStop(b);
   }

   void Mapper::InsertKeyFrame(KeyFrame * pKF)
   {
      mpLocalMapper->InsertKeyFrame(pKF);
   }
   
   void Mapper::GlobalBundleAdjustemnt(int nIterations, bool *pbStopFlag,
      const unsigned long nLoopKF, const bool bRobust)
   {
      Optimizer::GlobalBundleAdjustemnt(mpMap, nIterations, pbStopFlag, nLoopKF, bRobust);
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
}