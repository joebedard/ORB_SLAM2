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

   Mapper::Mapper(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular)
      : mpMap(pMap), mpVocab(pVocab), mbMonocular(bMonocular), mInitialized(false)
   {
      if (pMap == NULL)
         throw std::exception("pMap must not be NULL");

      mpKeyFrameDB = new KeyFrameDatabase(*pVocab);
   }

   std::mutex & Mapper::getMutexMapUpdate()
   {
      return mpMap->mMutexMapUpdate;
   }

   long unsigned Mapper::KeyFramesInMap()
   {
      return mpMap->KeyFramesInMap();
   }

   long unsigned int Mapper::NextMapPointId()
   {
      // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
      unique_lock<mutex> lock(mpMap->mMutexPointCreation);
      return nNextMapPointId++;
   }

   void Mapper::AddMapPoint(MapPoint * pMP)
   {
      mpMap->AddMapPoint(pMP);
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

   bool Mapper::stopRequested()
   {
      return mpLocalMapper->stopRequested();
   }
   
   bool Mapper::AcceptKeyFrames()
   {
      return mpLocalMapper->AcceptKeyFrames();
   }
   
   void Mapper::InsertKeyFrame(KeyFrame * pKF)
   {
      mpLocalMapper->InsertKeyFrame(pKF);
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

   KeyFrame * Mapper::CreateNewKeyFrame(Frame & currentFrame, ORB_SLAM2::eSensor sensorType)
   {
      if (!mpLocalMapper->SetNotStop(true))
         return NULL;

      // If the mapping accepts keyframes, insert keyframe.
      // Otherwise send a signal to interrupt BA
      if (!mpLocalMapper->AcceptKeyFrames())
      {
         mpLocalMapper->InterruptBA();
         if (sensorType != MONOCULAR)
         {
            if (mpLocalMapper->KeyframesInQueue() >= 3)
               return NULL;
         }
         else
            return NULL;
      }

      KeyFrame* pKF = new KeyFrame(currentFrame);

      if (sensorType != MONOCULAR)
      {
         currentFrame.UpdatePoseMatrices();

         // We sort points by the measured depth by the stereo/RGBD sensor.
         // We create all those MapPoints whose depth < mThDepth.
         // If there are less than 100 close points we create the 100 closest.
         vector<pair<float, int> > vDepthIdx;
         vDepthIdx.reserve(currentFrame.N);
         for (int i = 0; i < currentFrame.N; i++)
         {
            float z = currentFrame.mvDepth[i];
            if (z>0)
            {
               vDepthIdx.push_back(make_pair(z, i));
            }
         }

         if (!vDepthIdx.empty())
         {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j<vDepthIdx.size();j++)
            {
               int i = vDepthIdx[j].second;

               bool bCreateNew = false;

               MapPoint* pMP = currentFrame.mvpMapPoints[i];
               if (!pMP)
                  bCreateNew = true;
               else if (pMP->Observations()<1)
               {
                  bCreateNew = true;
                  currentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
               }

               if (bCreateNew)
               {
                  cv::Mat x3D = currentFrame.UnprojectStereo(i);
                  MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
                  pNewMP->AddObservation(pKF, i);
                  pKF->AddMapPoint(pNewMP, i);
                  pNewMP->ComputeDistinctiveDescriptors();
                  pNewMP->UpdateNormalAndDepth();
                  AddMapPoint(pNewMP);

                  currentFrame.mvpMapPoints[i] = pNewMP;
                  nPoints++;
               }
               else
               {
                  nPoints++;
               }

               if (vDepthIdx[j].first > currentFrame.mThDepth && nPoints > 100)
                  break;
            }
         }
      }

      InsertKeyFrame(pKF);

      mpLocalMapper->SetNotStop(false);

      return pKF;
   }

   void Mapper::Initialize(Map & pMap)
   {
      if (mInitialized)
         throw std::exception("The mapper may only be initialized once.");

      *mpMap = pMap;

      //Initialize the Local Mapping thread and launch
      mpLocalMapper = new LocalMapping(mpMap, mpKeyFrameDB, mbMonocular);
      mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

      //Initialize the Loop Closing thread and launch
      mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDB, mpVocab, !mbMonocular);
      mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

      mpLocalMapper->SetLoopCloser(mpLoopCloser);
      mpLoopCloser->SetLocalMapper(mpLocalMapper);

      mInitialized = true;
   }

}