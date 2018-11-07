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

#include "KeyFrame.h"
#include "Converter.h"
#include "Serializer.h"

#include <mutex>

namespace ORB_SLAM2
{

   KeyFrame::KeyFrame(id_type id, Frame & frame) 
      : SyncPrint("KeyFrame: ")

      // non-const variables
      , mnId(id)
      , mTimestamp(frame.mTimeStamp)
      , mFC(*frame.mFC)
      , N(frame.N)
      , mvKeys(frame.mvKeys)
      , mvKeysUn(frame.mvKeysUn)
      , mvuRight(frame.mvuRight)
      , mvDepth(frame.mvDepth)
      , mDescriptors(frame.mDescriptors.clone())
      , mBowVec(frame.mBowVec)
      , mFeatVec(frame.mFeatVec)
      , mnScaleLevels(frame.mnScaleLevels)
      , mfScaleFactor(frame.mfScaleFactor)
      , mfLogScaleFactor(frame.mfLogScaleFactor)
      , mvScaleFactors(frame.mvScaleFactors)
      , mvLevelSigma2(frame.mvLevelSigma2)
      , mvInvLevelSigma2(frame.mvInvLevelSigma2)
      , mvpMapPoints(frame.mvpMapPoints)
      , mnGridCols(FRAME_GRID_COLS)
      , mnGridRows(FRAME_GRID_ROWS)
      , mnTrackReferenceForFrame(0)
      , mnFuseTargetForKF(0)
      , mnBALocalForKF(0)
      , mnBAFixedForKF(0)
      , mnLoopQuery(0)
      , mnLoopWords(0)
      , mnRelocQuery(0)
      , mnRelocWords(0)
      , mnBAGlobalForKF(0)
      , mbFirstConnection(true)
      , mpParent(NULL)
      , mbNotErase(false)
      , mbToBeErased(false)
      , mbBad(false)

      // const references
      , timestamp(mTimestamp)
   {
      for (int i = 0;i < FRAME_GRID_COLS;i++)
         for (int j = 0; j < FRAME_GRID_ROWS; j++)
            mGrid[i][j] = frame.mGrid[i][j];

      SetPose(frame.mTcw);
   }

   id_type KeyFrame::GetId()
   {
      return mnId;
   }

   bool KeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
   {
      posX = round((kp.pt.x - mFC.minX) * mFC.gridElementWidthInv);
      posY = round((kp.pt.y - mFC.minY) * mFC.gridElementHeightInv);

      //Keypoint's coordinates are undistorted, which could cause to go out of the image
      if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
         return false;

      return true;
   }

   void KeyFrame::AssignFeaturesToGrid()
   {
      int nReserve = 0.5f*N / (FRAME_GRID_COLS*FRAME_GRID_ROWS);
      for (unsigned int i = 0; i < FRAME_GRID_COLS;i++)
         for (unsigned int j = 0; j < FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

      for (int i = 0;i < N;i++)
      {
         const cv::KeyPoint &kp = mvKeysUn[i];

         int nGridPosX, nGridPosY;
         if (PosInGrid(kp, nGridPosX, nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
      }
   }

   void KeyFrame::ComputeBoW(ORBVocabulary & vocab)
   {
      if (mBowVec.empty() || mFeatVec.empty())
      {
         vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
         // Feature vector associate features with nodes in the 4th level (from leaves up)
         // We assume the vocabulary tree has 6 levels, change the 4 otherwise
         vocab.transform(vCurrentDesc, mBowVec, mFeatVec, 4);
      }
   }

   void KeyFrame::SetPose(const cv::Mat &Tcw_)
   {
      unique_lock<mutex> lock(mMutexPose);
      Tcw_.copyTo(Tcw);
      cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
      cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
      cv::Mat Rwc = Rcw.t();
      Ow = -Rwc * tcw;

      Twc = cv::Mat::eye(4, 4, Tcw.type());
      Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
      Ow.copyTo(Twc.rowRange(0, 3).col(3));
   }

   cv::Mat KeyFrame::GetPose()
   {
      unique_lock<mutex> lock(mMutexPose);
      return Tcw.clone();
   }

   cv::Mat KeyFrame::GetPoseInverse()
   {
      unique_lock<mutex> lock(mMutexPose);
      return Twc.clone();
   }

   cv::Mat KeyFrame::GetCameraCenter()
   {
      unique_lock<mutex> lock(mMutexPose);
      return Ow.clone();
   }

   cv::Mat KeyFrame::GetRotation()
   {
      unique_lock<mutex> lock(mMutexPose);
      return Tcw.rowRange(0, 3).colRange(0, 3).clone();
   }

   cv::Mat KeyFrame::GetTranslation()
   {
      unique_lock<mutex> lock(mMutexPose);
      return Tcw.rowRange(0, 3).col(3).clone();
   }

   void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
   {
      {
         unique_lock<mutex> lock(mMutexConnections);
         if (!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF] = weight;
         else if (mConnectedKeyFrameWeights[pKF] != weight)
            mConnectedKeyFrameWeights[pKF] = weight;
         else
            return;
      }

      UpdateBestCovisibles();
   }

   void KeyFrame::UpdateBestCovisibles()
   {
      unique_lock<mutex> lock(mMutexConnections);
      vector<pair<int, KeyFrame*> > vPairs;
      vPairs.reserve(mConnectedKeyFrameWeights.size());
      for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
         vPairs.push_back(make_pair(mit->second, mit->first));

      sort(vPairs.begin(), vPairs.end());
      list<KeyFrame*> lKFs;
      list<int> lWs;
      for (size_t i = 0, iend = vPairs.size(); i < iend;i++)
      {
         lKFs.push_front(vPairs[i].second);
         lWs.push_front(vPairs[i].first);
      }

      mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
      mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
   }

   set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
   {
      unique_lock<mutex> lock(mMutexConnections);
      set<KeyFrame*> s;
      for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin();mit != mConnectedKeyFrameWeights.end();mit++)
         s.insert(mit->first);
      return s;
   }

   vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
   {
      unique_lock<mutex> lock(mMutexConnections);
      return mvpOrderedConnectedKeyFrames;
   }

   vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
   {
      unique_lock<mutex> lock(mMutexConnections);
      if ((int)mvpOrderedConnectedKeyFrames.size() < N)
         return mvpOrderedConnectedKeyFrames;
      else
         return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

   }

   vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
   {
      unique_lock<mutex> lock(mMutexConnections);

      if (mvpOrderedConnectedKeyFrames.empty())
         return vector<KeyFrame*>();

      vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);
      if (it == mvOrderedWeights.end())
         return vector<KeyFrame*>();
      else
      {
         int n = it - mvOrderedWeights.begin();
         return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
      }
   }

   int KeyFrame::GetWeight(KeyFrame *pKF)
   {
      unique_lock<mutex> lock(mMutexConnections);
      if (mConnectedKeyFrameWeights.count(pKF))
         return mConnectedKeyFrameWeights[pKF];
      else
         return 0;
   }

   void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
   {
      unique_lock<mutex> lock(mMutexFeatures);
      mvpMapPoints[idx] = pMP;
   }

   void KeyFrame::EraseMapPointMatch(const size_t &idx)
   {
      unique_lock<mutex> lock(mMutexFeatures);
      mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
   }

   void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
   {
      int idx = pMP->GetIndexInKeyFrame(this);
      if (idx >= 0)
         mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
   }


   void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
   {
      mvpMapPoints[idx] = pMP;
   }

   set<MapPoint*> KeyFrame::GetMapPoints()
   {
      unique_lock<mutex> lock(mMutexFeatures);
      set<MapPoint*> s;
      for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++)
      {
         if (!mvpMapPoints[i])
            continue;
         MapPoint* pMP = mvpMapPoints[i];
         if (!pMP->isBad())
            s.insert(pMP);
      }
      return s;
   }

   int KeyFrame::TrackedMapPoints(const int &minObs)
   {
      unique_lock<mutex> lock(mMutexFeatures);

      int nPoints = 0;
      const bool bCheckObs = minObs > 0;
      for (int i = 0; i < N; i++)
      {
         MapPoint* pMP = mvpMapPoints[i];
         if (pMP)
         {
            if (!pMP->isBad())
            {
               if (bCheckObs)
               {
                  if (mvpMapPoints[i]->Observations() >= minObs)
                     nPoints++;
               }
               else
                  nPoints++;
            }
         }
      }

      return nPoints;
   }

   vector<MapPoint*> KeyFrame::GetMapPointMatches()
   {
      unique_lock<mutex> lock(mMutexFeatures);
      return mvpMapPoints;
   }

   MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
   {
      unique_lock<mutex> lock(mMutexFeatures);
      return mvpMapPoints[idx];
   }

   void KeyFrame::UpdateConnections()
   {
      Print("begin UpdateConnections");
      map<KeyFrame*, int> KFcounter;

      vector<MapPoint*> vpMP;

      {
         unique_lock<mutex> lockMPs(mMutexFeatures);
         vpMP = mvpMapPoints;
      }

      //For all map points in keyframe check in which other keyframes are they seen
      //Increase counter for those keyframes
      for (vector<MapPoint*>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++)
      {
         MapPoint* pMP = *vit;

         if (!pMP)
            continue;

         if (pMP->isBad())
            continue;

         map<KeyFrame*, size_t> observations = pMP->GetObservations();

         for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
         {
            if (mit->first->mnId == mnId)
               continue;
            KFcounter[mit->first]++;
         }
      }

      // This should not happen
      if (KFcounter.empty())
      {
         Print("end UpdateConnections 1");
         return;
      }

      //If the counter is greater than threshold add connection
      //In case no keyframe counter is over threshold add the one with maximum counter
      int nmax = 0;
      KeyFrame* pKFmax = NULL;
      int th = 15;

      vector<pair<int, KeyFrame*> > vPairs;
      vPairs.reserve(KFcounter.size());
      for (map<KeyFrame*, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++)
      {
         if (mit->second > nmax)
         {
            nmax = mit->second;
            pKFmax = mit->first;
         }
         if (mit->second >= th)
         {
            vPairs.push_back(make_pair(mit->second, mit->first));
            (mit->first)->AddConnection(this, mit->second);
         }
      }

      if (vPairs.empty())
      {
         vPairs.push_back(make_pair(nmax, pKFmax));
         pKFmax->AddConnection(this, nmax);
      }

      sort(vPairs.begin(), vPairs.end());
      list<KeyFrame*> lKFs;
      list<int> lWs;
      for (size_t i = 0; i < vPairs.size();i++)
      {
         lKFs.push_front(vPairs[i].second);
         lWs.push_front(vPairs[i].first);
      }

      {
         unique_lock<mutex> lockCon(mMutexConnections);

         // mspConnectedKeyFrames = spConnectedKeyFrames;
         mConnectedKeyFrameWeights = KFcounter;
         mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
         mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

         if (mbFirstConnection && mnId != 0)
         {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
         }

      }
      Print("end UpdateConnections 2");
   }

   void KeyFrame::AddChild(KeyFrame *pKF)
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      mspChildrens.insert(pKF);
   }

   void KeyFrame::EraseChild(KeyFrame *pKF)
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      mspChildrens.erase(pKF);
   }

   void KeyFrame::ChangeParent(KeyFrame *pKF)
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      mpParent = pKF;
      pKF->AddChild(this);
   }

   set<KeyFrame*> KeyFrame::GetChilds()
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      return mspChildrens;
   }

   KeyFrame* KeyFrame::GetParent()
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      return mpParent;
   }

   bool KeyFrame::hasChild(KeyFrame *pKF)
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      return mspChildrens.count(pKF);
   }

   void KeyFrame::AddLoopEdge(KeyFrame *pKF)
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      mbNotErase = true;
      mspLoopEdges.insert(pKF);
   }

   set<KeyFrame*> KeyFrame::GetLoopEdges()
   {
      unique_lock<mutex> lockCon(mMutexConnections);
      return mspLoopEdges;
   }

   void KeyFrame::SetNotErase()
   {
      unique_lock<mutex> lock(mMutexConnections);
      mbNotErase = true;
   }

   bool KeyFrame::SetErase(Map* pMap, KeyFrameDatabase* pKeyFrameDB)
   {
      Print("begin SetErase");
      {
         unique_lock<mutex> lock(mMutexConnections);
         if (mspLoopEdges.empty())
         {
            mbNotErase = false;
         }
      }

      if (mbToBeErased)
      {
         Print("end SetErase 1");
         return SetBadFlag(pMap, pKeyFrameDB);
      }
      Print("end SetErase 2");
      return false;
   }

   bool KeyFrame::SetBadFlag(Map* pMap, KeyFrameDatabase* pKeyFrameDB)
   {
      {
         unique_lock<mutex> lock(mMutexConnections);
         if (mnId == 0)
            return false;
         else if (mbNotErase)
         {
            mbToBeErased = true;
            return false;
         }
      }

      for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
         mit->first->EraseConnection(this);

      for (size_t i = 0; i < mvpMapPoints.size(); i++)
      {
         if (mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this, pMap);
      }

      {
         unique_lock<mutex> lock(mMutexConnections);
         unique_lock<mutex> lock1(mMutexFeatures);

         mConnectedKeyFrameWeights.clear();
         mvpOrderedConnectedKeyFrames.clear();

         // Update Spanning Tree
         set<KeyFrame*> sParentCandidates;
         sParentCandidates.insert(mpParent);

         // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
         // Include that children as new parent candidate for the rest
         while (!mspChildrens.empty())
         {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for (set<KeyFrame*>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++)
            {
               KeyFrame* pKF = *sit;
               if (pKF->isBad())
                  continue;

               // Check if a parent candidate is connected to the keyframe
               vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
               for (size_t i = 0, iend = vpConnected.size(); i < iend; i++)
               {
                  for (set<KeyFrame*>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++)
                  {
                     if (vpConnected[i]->mnId == (*spcit)->mnId)
                     {
                        int w = pKF->GetWeight(vpConnected[i]);
                        if (w > max)
                        {
                           pC = pKF;
                           pP = vpConnected[i];
                           max = w;
                           bContinue = true;
                        }
                     }
                  }
               }
            }

            if (bContinue)
            {
               pC->ChangeParent(pP);
               sParentCandidates.insert(pC);
               mspChildrens.erase(pC);
            }
            else
               break;
         }

         // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
         if (!mspChildrens.empty())
            for (set<KeyFrame*>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++)
            {
               (*sit)->ChangeParent(mpParent);
            }

         mpParent->EraseChild(this);
         mTcp = Tcw * mpParent->GetPoseInverse();
         mbBad = true;
      }


      pMap->EraseKeyFrame(this);
      pKeyFrameDB->erase(this);
      return true;
   }

   bool KeyFrame::isBad()
   {
      unique_lock<mutex> lock(mMutexConnections);
      return mbBad;
   }

   void KeyFrame::EraseConnection(KeyFrame* pKF)
   {
      bool bUpdate = false;
      {
         unique_lock<mutex> lock(mMutexConnections);
         if (mConnectedKeyFrameWeights.count(pKF))
         {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate = true;
         }
      }

      if (bUpdate)
         UpdateBestCovisibles();
   }

   vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
   {
      vector<size_t> vIndices;
      vIndices.reserve(N);

      const int nMinCellX = max(0, (int)floor((x - mFC.minX - r) * mFC.gridElementWidthInv));
      if (nMinCellX >= mnGridCols)
         return vIndices;

      const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mFC.minX + r) * mFC.gridElementWidthInv));
      if (nMaxCellX < 0)
         return vIndices;

      const int nMinCellY = max(0, (int)floor((y - mFC.minY - r) * mFC.gridElementHeightInv));
      if (nMinCellY >= mnGridRows)
         return vIndices;

      const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mFC.minY + r) * mFC.gridElementHeightInv));
      if (nMaxCellY < 0)
         return vIndices;

      for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
      {
         for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
         {
            const vector<size_t> vCell = mGrid[ix][iy];
            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
               const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
               const float distx = kpUn.pt.x - x;
               const float disty = kpUn.pt.y - y;

               if (fabs(distx) < r && fabs(disty) < r)
                  vIndices.push_back(vCell[j]);
            }
         }
      }

      return vIndices;
   }

   bool KeyFrame::IsInImage(const float &x, const float &y) const
   {
      return (x >= mFC.minX && x < mFC.maxX && y >= mFC.minY && y < mFC.maxY);
   }

   cv::Mat KeyFrame::UnprojectStereo(int i)
   {
      const float z = mvDepth[i];
      if (z > 0)
      {
         const float u = mvKeys[i].pt.x;
         const float v = mvKeys[i].pt.y;
         const float x = (u - mFC.cx) * z * mFC.invfx;
         const float y = (v - mFC.cy) * z * mFC.invfy;
         cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

         unique_lock<mutex> lock(mMutexPose);
         return Twc.rowRange(0, 3).colRange(0, 3)*x3Dc + Twc.rowRange(0, 3).col(3);
      }
      else
         return cv::Mat();
   }

   float KeyFrame::ComputeSceneMedianDepth(const int q)
   {
      vector<MapPoint*> vpMapPoints;
      cv::Mat Tcw_;
      {
         unique_lock<mutex> lock(mMutexFeatures);
         unique_lock<mutex> lock2(mMutexPose);
         vpMapPoints = mvpMapPoints;
         Tcw_ = Tcw.clone();
      }

      vector<float> vDepths;
      vDepths.reserve(N);
      cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
      Rcw2 = Rcw2.t();
      float zcw = Tcw_.at<float>(2, 3);
      for (int i = 0; i < N; i++)
      {
         if (mvpMapPoints[i])
         {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
         }
      }

      sort(vDepths.begin(), vDepths.end());

      return vDepths[(vDepths.size() - 1) / q];
   }

   size_t KeyFrame::GetBufferSize()
   {
      unsigned int size = sizeof(KeyFrame::Header);
      size += Serializer::GetKeyPointVectorBufferSize(mvKeys);
      size += Serializer::GetKeyPointVectorBufferSize(mvKeysUn);
      size += Serializer::GetVectorBufferSize<float>(mvuRight);
      size += Serializer::GetVectorBufferSize<float>(mvDepth);
      size += Serializer::GetMatBufferSize(mDescriptors);
      //mBowVec // will be re-created by LocalMapping::ProcessNewKeyFrame->KeyFrame::ComputeBow
      //mFeatVec // will be re-created by LocalMapping::ProcessNewKeyFrame->KeyFrame::ComputeBow
      size += Serializer::GetMatBufferSize(mTcp);
      size += Serializer::GetVectorBufferSize<float>(mvScaleFactors);
      size += Serializer::GetVectorBufferSize<float>(mvLevelSigma2);
      size += Serializer::GetVectorBufferSize<float>(mvInvLevelSigma2);
      size += Serializer::GetMatBufferSize(Tcw);
      size += Serializer::GetMatBufferSize(Twc);
      size += Serializer::GetMatBufferSize(Ow);
      size += sizeof(size_t) + mvpMapPoints.size() * sizeof(id_type);
      size += sizeof(size_t) + mConnectedKeyFrameWeights.size() * sizeof(KeyFrameWeight);
      size += sizeof(size_t) + mvpOrderedConnectedKeyFrames.size() * sizeof(id_type);
      size += Serializer::GetVectorBufferSize<int>(mvOrderedWeights);
      size += sizeof(size_t) + mspChildrens.size() * sizeof(id_type);
      size += sizeof(size_t) + mspLoopEdges.size() * sizeof(id_type);
      return size;
   }

   void * KeyFrame::ReadBytes(const void * data, Map & map)
   {
      AssignFeaturesToGrid();
      return NULL;
   }

   void * KeyFrame::WriteBytes(const void * data)
   {
      return NULL;
   }

} //namespace ORB_SLAM
