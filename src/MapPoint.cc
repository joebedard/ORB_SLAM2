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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "Serializer.h"

#include<mutex>

namespace ORB_SLAM2
{
   using namespace std;

   mutex MapPoint::mGlobalMutex;

   MapPoint::MapPoint(id_type id)
      : SyncPrint("MapPoint: ")
      , mnId(id)
      , mnFirstKFid(-1)
      , nObs(0)
      , mnTrackReferenceForFrame(0)
      , mnLastFrameSeen(0)
      , mnBALocalForKF(0)
      , mnFuseCandidateForKF(0)
      , mnLoopPointForKF(0)
      , mnCorrectedByKF(0)
      , mnCorrectedReference(0)
      , mnBAGlobalForKF(0)
      , mpRefKF(static_cast<KeyFrame*>(NULL))
      , mnVisible(1)
      , mnFound(1)
      , mbBad(false)
      , mpReplaced(static_cast<MapPoint*>(NULL))
      , mfMinDistance(0)
      , mfMaxDistance(0)
      , mModified(true)

      // public read-only access to private variables
      , firstKFid(mnFirstKFid)
   {
   }

   MapPoint::MapPoint(id_type id, const cv::Mat & worldPos, KeyFrame *pRefKF) 
      : SyncPrint("MapPoint: ")
      , mnId(id)
      , mnFirstKFid(pRefKF->GetId())
      , nObs(0)
      , mnTrackReferenceForFrame(0)
      , mnLastFrameSeen(0)
      , mnBALocalForKF(0)
      , mnFuseCandidateForKF(0)
      , mnLoopPointForKF(0)
      , mnCorrectedByKF(0)
      , mnCorrectedReference(0)
      , mnBAGlobalForKF(0)
      , mpRefKF(pRefKF)
      , mnVisible(1)
      , mnFound(1)
      , mbBad(false)
      , mpReplaced(static_cast<MapPoint*>(NULL))
      , mfMinDistance(0)
      , mfMaxDistance(0)
      , mNormalVector(cv::Mat::zeros(3, 1, CV_32F))
      , mWorldPos(worldPos)
      , mModified(true)

      // public read-only access to private variables
      , firstKFid(mnFirstKFid)
   {
      if (worldPos.empty())
         throw exception("MapPoint::SetWorldPos([])!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
   }

   void MapPoint::PrintPrefix(ostream & out)
   {
      SyncPrint::PrintPrefix(out);
      out << "id=" << mnId << " ";
   }

   void MapPoint::SetWorldPos(const cv::Mat &Pos)
   {
      unique_lock<mutex> lock2(mGlobalMutex);
      unique_lock<mutex> lock(mMutexPos);
      if (Pos.empty())
         throw exception("MapPoint::SetWorldPos([])!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Pos.copyTo(mWorldPos);
      SetModified(true);
   }

   cv::Mat MapPoint::GetWorldPos()
   {
      unique_lock<mutex> lock(mMutexPos);
      return mWorldPos.clone();
   }

   cv::Mat MapPoint::GetNormal()
   {
      unique_lock<mutex> lock(mMutexPos);
      return mNormalVector.clone();
   }

   KeyFrame* MapPoint::GetReferenceKeyFrame()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return mpRefKF;
   }

   void MapPoint::Link(KeyFrame & rKF, size_t idx) {
      size_t prevIdx = -1;
      unique_lock<recursive_mutex> lock(mMutexObservations);
      if (mObservations.count(&rKF) > 0) {
         prevIdx = mObservations[&rKF];
         if (prevIdx == idx)
            return;
         rKF.Unlink(prevIdx);
      }
      mObservations[&rKF] = idx;
      try {
         // an invalid idx could cause an exception
         rKF.Link(*this, idx);
      }
      catch (exception & e) {
         if (prevIdx > -1)
            mObservations[&rKF] = prevIdx;
         else
            mObservations.erase(&rKF);
         throw e;
      }

      // at this point, all linking is successful and complete
      SetModified(true);
      if (rKF.right[idx] >= 0)
         nObs += 2;
      else
         nObs++;
      if (mpRefKF = NULL)
         mpRefKF = &rKF;
   }

   void MapPoint::Unlink(KeyFrame & rKF) {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      if (mObservations.count(&rKF) > 0) {
         size_t prevIdx = mObservations[&rKF];
         mObservations.erase(&rKF);
         try {
            rKF.Unlink(prevIdx);
         }
         catch (exception & e) {
            mObservations[&rKF] = prevIdx;
            throw e;
         }

         // at this point, all unlinking is successful and complete
         SetModified(true);
         if (rKF.right[prevIdx] >= 0)
            nObs -= 2;
         else
            nObs--;
         if (mpRefKF == &rKF) {
            if (mObservations.empty())
               mpRefKF = NULL;
            else
               mpRefKF = mObservations.begin()->first;
         }
         // If only 2 observations or less, discard point
         if (nObs <= 2)
            SetBadFlag(NULL);
      }
   }

   void MapPoint::ReplaceWith(MapPoint & rMP) {
      if (rMP.mnId == this->mnId)
         return;
      unique_lock<recursive_mutex> lock(mMutexObservations);
      unordered_map<KeyFrame *, size_t> obs = mObservations;
      if (obs.size() == 0)
         return;
      for (pair<KeyFrame *, size_t> p : obs) {
         p.first->Link(rMP, p.second);
      }
      mbBad = true;
      mpReplaced = &rMP;
      rMP.IncreaseFound(mnFound);
      rMP.IncreaseVisible(mnVisible);
      rMP.ComputeDistinctiveDescriptors();
   }

   //void MapPoint::EraseObservation(KeyFrame * pKF, Map * pMap)
   //{
   //   Print("begin EraseObservation");
   //   bool bBad = false;
   //   int prevIdx = -1;
   //   {
   //      unique_lock<recursive_mutex> lock(mMutexObservations);
   //      if (!mObservations.count(pKF))
   //         return;

   //      prevIdx = mObservations[pKF];
   //      mObservations.erase(pKF);
   //      SetModified(true);

   //      if (pKF->right[prevIdx] >= 0)
   //         nObs -= 2;
   //      else
   //         nObs--;

   //      if (mpRefKF == pKF)
   //      {
   //         if (mObservations.empty())
   //            mpRefKF = NULL;
   //         else
   //            mpRefKF = mObservations.begin()->first;
   //      }

   //      // If only 2 observations or less, discard point
   //      if (nObs <= 2)
   //      {
   //         bBad = true;
   //      }
   //   }

   //   if (bBad)
   //      SetBadFlag(pMap);

   //   if (prevIdx >= 0)
   //      pKF->EraseMapPointMatch(prevIdx);

   //   Print("end EraseObservation");
   //}

   unordered_map<KeyFrame*, size_t> MapPoint::GetObservations()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return mObservations;
   }

   int MapPoint::Observations()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return nObs;
   }

   void MapPoint::SetBadFlag(Map * pMap)
   {
      Print("begin SetBadFlag");
      unordered_map<KeyFrame*, size_t> obs;
      {
         unique_lock<recursive_mutex> lock1(mMutexObservations);
         unique_lock<mutex> lock2(mMutexPos);
         mbBad = true;
         obs = mObservations;
         mObservations.clear();
         mpRefKF == NULL;
         SetModified(true);
      }
      for (unordered_map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
      {
         KeyFrame * pKF = mit->first;
         stringstream ss3;
         ss3 << pKF->GetId() << "->Unlink(" << mit->second << ")";
         Print(ss3);
         //pKF->EraseMapPointMatch(mit->second);
         pKF->Unlink(mit->second);
      }

      //if (pMap)
      //   pMap->EraseMapPoint(this);
      Print("end SetBadFlag");
   }

   MapPoint * MapPoint::GetReplaced()
   {
      unique_lock<recursive_mutex> lock1(mMutexObservations);
      unique_lock<mutex> lock2(mMutexPos);
      return mpReplaced;
   }

   MapPoint * MapPoint::FindFinalReplacement(MapPoint * pMP)
   {
      if (!pMP)
         throw exception("MapPoint::FindFinalReplacement pMP == NULL");

      // search through replacement points until it finds the last one
      MapPoint * pNext = pMP->GetReplaced();
      while (pNext)
      {
         pMP = pNext;
         pNext = pMP->GetReplaced();
      }
      return pMP;
   }

   //void MapPoint::Replace(MapPoint* pMP, Map * pMap)
   //{
   //   if (!pMP)
   //      throw exception("MapPoint::Replace pMP == NULL");

   //   if (pMP->mnId == this->mnId)
   //      return;

   //   if (pMap)
   //   {
   //      //pMap->ReplaceMapPoint(this->mnId, pMP);
   //      //pMap->EraseMapPoint(this);
   //   }

   //   int nvisible, nfound;
   //   unordered_map<KeyFrame*, size_t> obs;
   //   {
   //      unique_lock<recursive_mutex> lock1(mMutexObservations);
   //      unique_lock<mutex> lock2(mMutexPos);
   //      obs = mObservations;
   //      mObservations.clear();
   //      mbBad = true;
   //      nvisible = mnVisible;
   //      nfound = mnFound;
   //      mpReplaced = pMP;
   //      SetModified(true);
   //   }

   //   for (unordered_map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
   //   {
   //      // Replace measurement in keyframe
   //      KeyFrame* pKF = mit->first;

   //      if (!pMP->IsObserving(pKF))
   //      {
   //         pKF->ReplaceMapPointMatch(mit->second, pMP);
   //         pMP->AddObservation(pKF, mit->second);
   //      }
   //      else
   //      {
   //         pKF->EraseMapPointMatch(mit->second);
   //      }
   //   }
   //   pMP->IncreaseFound(nfound);
   //   pMP->IncreaseVisible(nvisible);
   //   pMP->ComputeDistinctiveDescriptors();
   //}

   bool MapPoint::isBad()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      unique_lock<mutex> lock2(mMutexPos);
      return mbBad;
   }

   void MapPoint::IncreaseVisible(int n)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      mnVisible += n;
      SetModified(true);
   }

   void MapPoint::IncreaseFound(int n)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      mnFound += n;
      SetModified(true);
   }

   float MapPoint::GetFoundRatio()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return static_cast<float>(mnFound) / mnVisible;
   }

   void MapPoint::ComputeDistinctiveDescriptors()
   {
      //Print("begin ComputeDistinctiveDescriptors");
      // Retrieve all observed descriptors
      vector<cv::Mat> vDescriptors;

      unordered_map<KeyFrame*, size_t> obs;

      {
         unique_lock<recursive_mutex> lock1(mMutexObservations);
         if (mbBad)
         {
            //Print("end ComputeDistinctiveDescriptors 1");
            return;
         }
         obs = mObservations;
      }

      if (obs.empty())
      {
         //Print("end ComputeDistinctiveDescriptors 2");
         return;
      }

      vDescriptors.reserve(obs.size());

      //Print("1");
      for (unordered_map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
      {
         KeyFrame* pKF = mit->first;

         if (!pKF->isBad())
         {
            //stringstream ss; ss << pKF->descriptors.row(mit->second); Print(ss);
            vDescriptors.push_back(pKF->descriptors.row(mit->second));
         }
      }

      if (vDescriptors.empty())
      {
         //Print("end ComputeDistinctiveDescriptors 3");
         return;
      }

      // Compute distances between them
      const size_t N = vDescriptors.size();

      //Print("2");
      //float Distances[N][N];
      float * Distances = new float[N*N];
      for (size_t i = 0;i < N;i++)
      {
         //Distances[i][i]=0;
         Distances[i*N + i] = 0;
         for (size_t j = i + 1;j < N;j++)
         {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            //Distances[i][j]=distij;
            Distances[i*N + j] = distij;
            //Distances[j][i]=distij;
            Distances[j*N + i] = distij;
         }
      }

      //Print("3");
      // Take the descriptor with least median distance to the rest
      int BestMedian = INT_MAX;
      int BestIdx = 0;
      for (size_t i = 0;i < N;i++)
      {
         //vector<int> vDists(Distances[i],Distances[i]+N);
         vector<int> vDists(&Distances[i*N], &Distances[i*N] + N);
         sort(vDists.begin(), vDists.end());
         int median = vDists[0.5*(N - 1)];

         if (median < BestMedian)
         {
            BestMedian = median;
            BestIdx = i;
         }
      }

      delete[] Distances;

      {
         unique_lock<recursive_mutex> lock(mMutexObservations);
         //Print("mDescriptor = vDescriptors[BestIdx].clone();");
         mDescriptor = vDescriptors[BestIdx].clone();
         SetModified(true);
      }
      //Print("end ComputeDistinctiveDescriptors");
   }

   cv::Mat MapPoint::GetDescriptor()
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return mDescriptor.clone();
   }

   int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      if (mObservations.count(pKF))
         return mObservations[pKF];
      else
         return -1;
   }

   bool MapPoint::IsObserving(KeyFrame *pKF)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      return (mObservations.count(pKF) > 0);
   }

   void MapPoint::UpdateNormalAndDepth()
   {
      //Print("begin UpdateNormalAndDepth");
      unordered_map<KeyFrame*, size_t> obs;
      KeyFrame* pRefKF;
      cv::Mat Pos;
      {
         unique_lock<recursive_mutex> lock1(mMutexObservations);
         unique_lock<mutex> lock2(mMutexPos);
         if (mbBad)
         {
            //Print("end UpdateNormalAndDepth 1");
            return;
         }
         obs = mObservations;
         pRefKF = mpRefKF;
         Pos = mWorldPos.clone();
      }

      if (!obs.empty())
      {
         //Print("2");
         cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
         int n = 0;
         for (unordered_map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
         {
            KeyFrame* pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali / cv::norm(normali);
            n++;
         }

         //Print("3");
         if (pRefKF == NULL)
            Print(string("pRefKF == NULL, MapPoint id=") + to_string(mnId));
         cv::Mat PC = Pos - pRefKF->GetCameraCenter();
         const float dist = cv::norm(PC);
         const int level = pRefKF->keysUn[obs[pRefKF]].octave;
         const float levelScaleFactor = pRefKF->scaleFactors[level];
         const int nLevels = pRefKF->scaleLevels;

         //Print("4");
         {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->scaleFactors[nLevels - 1];
            mNormalVector = normal / n;
            SetModified(true);
         }
      }
      //Print("end UpdateNormalAndDepth 2");
   }

   float MapPoint::GetMinDistanceInvariance()
   {
      unique_lock<mutex> lock(mMutexPos);
      return 0.8f*mfMinDistance;
   }

   float MapPoint::GetMaxDistanceInvariance()
   {
      unique_lock<mutex> lock(mMutexPos);
      return 1.2f*mfMaxDistance;
   }

   int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
   {
      float ratio;
      {
         unique_lock<mutex> lock(mMutexPos);
         ratio = mfMaxDistance / currentDist;
      }

      int nScale = ceil(log(ratio) / pKF->logScaleFactor);
      if (nScale < 0)
         nScale = 0;
      else if (nScale >= pKF->scaleLevels)
         nScale = pKF->scaleLevels - 1;

      return nScale;
   }

   int MapPoint::PredictScale(const float &currentDist, Frame* pF)
   {
      float ratio;
      {
         unique_lock<mutex> lock(mMutexPos);
         ratio = mfMaxDistance / currentDist;
      }

      int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
      if (nScale < 0)
         nScale = 0;
      else if (nScale >= pF->mnScaleLevels)
         nScale = pF->mnScaleLevels - 1;

      return nScale;
   }

   id_type MapPoint::GetId()
   {
      return mnId;
   }

   MapPoint * MapPoint::Find(const id_type id, Map & map, std::unordered_map<id_type, MapPoint *> & newMapPoints)
   {
      MapPoint * pMP = map.GetMapPoint(id);
      if (pMP == NULL)
         pMP = newMapPoints.count(id) ? newMapPoints.at(id) : NULL;
      return pMP;
   }

   id_type MapPoint::PeekId(void * const buffer)
   {
      MapPoint::Header * pHeader = (MapPoint::Header *)buffer;
      return pHeader->mnId;
   }

   size_t MapPoint::GetVectorBufferSize(const std::vector<MapPoint *> & mpv)
   {
      size_t size = sizeof(size_t);
      for (MapPoint * pMP : mpv)
         size += pMP->GetBufferSize();
      return size;
   }

   void * MapPoint::ReadVector(
      void * buffer, 
      Map & map, 
      std::unordered_map<id_type, KeyFrame *> & newKeyFrames, 
      std::unordered_map<id_type, MapPoint *> & newMapPoints, 
      std::vector<MapPoint *> & mpv)
   {
      SyncPrint::Print("MapPoint: ", "begin ReadVector");
      size_t quantityMPs;
      void * pData = Serializer::ReadValue<size_t>(buffer, quantityMPs);
      mpv.resize(quantityMPs);
      for (int i = 0; i < quantityMPs; ++i)
      {
         id_type id = MapPoint::PeekId(pData);
         MapPoint * pMP = MapPoint::Find(id, map, newMapPoints);
         if (pMP == NULL)
         {
            pMP = new MapPoint(id);
            newMapPoints[id] = pMP;
         }
         pData = pMP->ReadBytes(pData, map, newKeyFrames, newMapPoints);
         mpv.at(i) = pMP;
      }
      SyncPrint::Print("MapPoint: ", "end ReadVector");
      return pData;
   }

   void * MapPoint::WriteVector(
      void * buffer,
      std::vector<MapPoint *> & mpv)
   {
      void * pData = Serializer::WriteValue<size_t>(buffer, mpv.size());
      for (MapPoint * pMP : mpv)
      {
         pData = pMP->WriteBytes(pData);
      }
      return pData;
   }

   size_t MapPoint::GetSetBufferSize(const std::set<MapPoint *> & mps)
   {
      size_t size = sizeof(size_t);
      for (MapPoint * pMP : mps)
         size += pMP->GetBufferSize();
      return size;
   }

   void * MapPoint::ReadSet(
      void * buffer, 
      Map & map, 
      std::unordered_map<id_type, KeyFrame *> & newKeyFrames, 
      std::unordered_map<id_type, MapPoint *> & newMapPoints, 
      std::set<MapPoint *> & mps)
   {
      SyncPrint::Print("MapPoint: ", "begin ReadSet");
      size_t quantityMPs;
      void * pData = Serializer::ReadValue<size_t>(buffer, quantityMPs);
      mps.clear();
      for (int i = 0; i < quantityMPs; ++i)
      {
         id_type id = MapPoint::PeekId(pData);
         MapPoint * pMP = MapPoint::Find(id, map, newMapPoints);
         if (pMP == NULL)
         {
            pMP = new MapPoint(id);
            newMapPoints[id] = pMP;
         }
         pData = pMP->ReadBytes(pData, map, newKeyFrames, newMapPoints);
         mps.insert(pMP);
      }
      SyncPrint::Print("MapPoint: ", "begin ReadSet");
      return pData;
   }

   void * MapPoint::WriteSet(
      void * buffer,
      std::set<MapPoint *> & mps)
   {
      void * pData = Serializer::WriteValue<size_t>(buffer, mps.size());
      for (MapPoint * pMP : mps)
      {
         pData = pMP->WriteBytes(pData);
      }
      return pData;
   }

   size_t MapPoint::GetBufferSize()
   {
      //Print("begin GetBufferSize");
      size_t size = sizeof(MapPoint::Header);
      //Print("size += Serializer::GetMatBufferSize(mWorldPos);");
      size += Serializer::GetMatBufferSize(mWorldPos);
      //Print("size += Serializer::GetMatBufferSize(mNormalVector);");
      size += Serializer::GetMatBufferSize(mNormalVector);
      //Print("size += Serializer::GetMatBufferSize(mDescriptor);");
      //size += Serializer::GetMatBufferSize(mDescriptor);
      //Print("size += Serializer::GetVectorBufferSize<Observation>(mObservations.size());");
      size += Serializer::GetVectorBufferSize<Observation>(mObservations.size());
      //Print("end GetBufferSize");
      return size;
   }

   void * MapPoint::ReadBytes(
      void * const buffer, 
      Map & map, 
      std::unordered_map<id_type, KeyFrame *> & newKeyFrames, 
      std::unordered_map<id_type, MapPoint *> & newMapPoints)
   {
      //Print("begin ReadBytes");
      MapPoint::Header * pHeader = (MapPoint::Header *)buffer;
      if (mnId != pHeader->mnId)
         throw exception("MapPoint::ReadBytes mnId != pHeader->mnId");

      mnFirstKFid = pHeader->mnFirstKFId;
      nObs = pHeader->nObs;

      if (pHeader->mpRefKFId == (id_type)-1)
         mpRefKF = NULL;
      else
      {
         mpRefKF = KeyFrame::Find(pHeader->mpRefKFId, map, newKeyFrames);
         if (mpRefKF == NULL)
         {
            //Print("ReadBytes mpRefKF == NULL");
            std::stringstream ss;
            ss << "MapPoint::ReadBytes detected an unknown reference KeyFrame with id=" << pHeader->mpRefKFId;
            throw exception(ss.str().c_str());

            // this is a new MapPoint in a new KeyFrame, but the KeyFrame was not created yet
            // create the placeholder KeyFrame here, but call KeyFrame::ReadBytes later
            // see: server::InsertKeyFrame and MapChangeEvent::ReadBytes
            //mpRefKF = new KeyFrame(pHeader->mpRefKFId);
            //Print(string("created new KeyFrame with id=") + to_string(pHeader->mpRefKFId));
            //newKeyFrames[pHeader->mpRefKFId] = mpRefKF;
         }
      }

      mnVisible = pHeader->mnVisible;
      mnFound = pHeader->mnFound;
      mbBad = pHeader->mbBad;
      if (pHeader->mpReplacedId == (id_type)-1)
         mpReplaced = NULL;
      else
      {
         mpReplaced = MapPoint::Find(pHeader->mpReplacedId, map, newMapPoints);
         if (mpReplaced == NULL)
         {
            mpReplaced = new MapPoint(pHeader->mpReplacedId);
            newMapPoints[pHeader->mpReplacedId] = mpReplaced;
         }
      }
      mfMinDistance = pHeader->mfMinDistance;
      mfMaxDistance = pHeader->mfMaxDistance;

      // read variable-length data
      void * pData = pHeader + 1;
      //Print("1");
      pData = Serializer::ReadMatrix(pData, mWorldPos);
      //Print("2");
      pData = Serializer::ReadMatrix(pData, mNormalVector);
      //pData = Serializer::ReadMatrix(pData, mDescriptor);
      //Print("3");
      pData = ReadObservations(pData, map, newKeyFrames);

      if (mpRefKF && mpRefKF->isBad())
      {
         // the KeyFrame was deleted by the server, and the client did not know yet
         Print("the reference KeyFrame was deleted by the server, and the client did not know yet");
         Print(string("MapPoint id=") + to_string(mnId));
         if (mObservations.empty())
            mpRefKF == NULL;
         else
            mpRefKF = mObservations.begin()->first;
      }

      //Print("4");
      ComputeDistinctiveDescriptors();

      //Print("end ReadBytes");
      return pData;
   }

   void * MapPoint::WriteBytes(void * const buffer)
   {
      //string msg = string("WriteBytes id=") + to_string(GetId());
      //Print(msg);
      MapPoint::Header * pHeader = (MapPoint::Header *)buffer;
      //Print("1");
      pHeader->mnId = mnId;
      //Print("2");
      pHeader->mnFirstKFId = mnFirstKFid;
      //Print("3");
      pHeader->nObs = nObs;
      //Print("4");
      pHeader->mpRefKFId = mpRefKF == NULL ? (id_type)-1 : mpRefKF->GetId();
      //Print("5");
      pHeader->mnVisible = mnVisible;
      //Print("6");
      pHeader->mnFound = mnFound;
      //Print("7");
      pHeader->mbBad = mbBad;
      if (mpReplaced)
      {
         //Print("8");
         pHeader->mpReplacedId = mpReplaced->GetId();
      }
      else
      {
         //Print("9");
         pHeader->mpReplacedId = (id_type)-1;
      }
      //Print("10");
      pHeader->mfMinDistance = mfMinDistance;
      //Print("11");
      pHeader->mfMaxDistance = mfMaxDistance;

      // write variable-length data
      void * pData = pHeader + 1;
      //Print("12");
      pData = Serializer::WriteMatrix(pData, mWorldPos);
      //Print("13");
      pData = Serializer::WriteMatrix(pData, mNormalVector);
      //pData = Serializer::WriteMatrix(pData, mDescriptor);
      //Print("14");
      pData = WriteObservations(pData);
      return pData;
   }

   void * MapPoint::ReadObservations(
      void * const buffer,
      Map & map,
      std::unordered_map<id_type, KeyFrame *> & newKeyFrames)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      mObservations.clear();
      size_t * pQuantity = (size_t *)buffer;
      Observation * pData = (Observation *)(pQuantity + 1);
      Observation * pEnd = pData + *pQuantity;
      while (pData < pEnd)
      {
         KeyFrame * pKF = KeyFrame::Find(pData->keyFrameId, map, newKeyFrames);
         if (pKF == NULL)
         {
            std::stringstream ss;
            ss << "ReadObservations detected an unknown KeyFrame with id=" << pData->keyFrameId;
            Print(ss);
            throw exception(ss.str().c_str());

            // this is a new MapPoint in a new KeyFrame, but the KeyFrame was not created yet
            // create the placeholder KeyFrame here, but call KeyFrame::ReadBytes later
            // see: server::InsertKeyFrame and MapChangeEvent::ReadBytes
            //pKF = new KeyFrame(pData->keyFrameId);
            //newKeyFrames[pData->keyFrameId] = pKF;
         }
         else if (!pKF->isBad())
         {
            // the server remembers which KeyFrames it deleted, this is not one of them
            mObservations[pKF] = pData->index;
         }
         ++pData;
      }
      return pData;
   }

   void * MapPoint::WriteObservations(void * const buffer)
   {
      unique_lock<recursive_mutex> lock(mMutexObservations);
      size_t * pQuantity = (size_t *)buffer;
      *pQuantity = mObservations.size();
      Observation * pData = (Observation *)(pQuantity + 1);
      for (std::pair<KeyFrame *, size_t> p : mObservations)
      {
         pData->keyFrameId = p.first->GetId();
         pData->index = p.second;
         ++pData;
      }
      return pData;
   }

   bool MapPoint::GetModified()
   {
      unique_lock<mutex> lock(mMutexModified);
      return mModified;
   }
   
   void MapPoint::SetModified(bool b)
   {
      unique_lock<mutex> lock(mMutexModified);
      mModified = b;
   }

} //namespace ORB_SLAM
