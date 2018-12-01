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

#include "LocalMapping.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Sleep.h"

#include<mutex>

namespace ORB_SLAM2
{

   LocalMapping::LocalMapping(
      Map & map,
      std::mutex & mutexMapUpdate, 
      KeyFrameDatabase & keyFrameDB,
      ORBVocabulary & vocab,
      const float bMonocular,
      unsigned long firstMapPointId,
      unsigned int mapPointIdSpan
   ) :
      SyncPrint("LocalMapping: "),
      mMap(map),
      mMutexMapUpdate(mutexMapUpdate),
      mKeyFrameDB(keyFrameDB),
      mVocab(vocab),
      mbMonocular(bMonocular),
      mNextMapPointId(firstMapPointId),
      mMapPointIdSpan(mapPointIdSpan),
      mbResetRequested(false),
      mbFinishRequested(false),
      mbFinished(true),
      mbAbortBA(false),
      mbPaused(false),
      mbPauseRequested(false),
      mbNotPause(false),
      mbAcceptKeyFrames(true)
   {
   }

   void LocalMapping::SetLoopCloser(LoopClosing * pLoopCloser)
   {
      mpLoopCloser = pLoopCloser;
   }

   void LocalMapping::Run() try
   {
      Print("begin Run");
      mbFinished = false;

      while (1)
      {
         // Check if there are keyframes in the queue
         //Print("if (CheckNewKeyFrames())");
         if (CheckNewKeyFrames())
         {
            Print("new KeyFrame in LocalMapping");
            MapChangeEvent mapChanges;

            // Tracking will see that Local Mapping is busy
            Print("SetAcceptKeyFrames(false);");
            SetAcceptKeyFrames(false);

            // compute BoW, insert KF into map, update KF connections
            ProcessNewKeyFrame(mapChanges);

            // Check recent MapPoints
            MapPointCulling(mapChanges);

            // Triangulate new MapPoints
            CreateNewMapPoints(mapChanges);

            if (!CheckNewKeyFrames())
            {
               // Find more matches in neighbor keyframes and fuse point duplications
               // updates mpCurrentKeyFrame and neighbor keyframes, deletes (replaces) points, updates points
               SearchInNeighbors(mapChanges);
            }

            mbAbortBA = false;

            Print("if(!CheckNewKeyFrames() && !PauseRequested())");
            if (!CheckNewKeyFrames() && !PauseRequested())
            {
               // Local BA
               if (mMap.KeyFramesInMap() > 2)
               {
                  // deletes points, updates keyframes, updates points
                  Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mMap, mMutexMapUpdate, mapChanges);
               }

               // Check for redundant local Keyframes, and delete them
               KeyFrameCulling(mapChanges);
            }

            // mpCurrentKeyFrame added/updated, 0..N keyframes updated, 0..N keyframes deleted, 0..N points added, 0..N points updated, 0..N points deleted
            // TODO OK - notify map changes
            NotifyMapChanged(mapChanges);

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

            // Tracking will see that Local Mapping is not busy
            Print("SetAcceptKeyFrames(true);");
            SetAcceptKeyFrames(true);
         }
         else if (Pause())
         {
            // Safe area to stop
            Print("PAUSE");
            while (IsPaused() && !CheckFinish())
            {
               sleep(3000);
            }
            if (CheckFinish())
               break;
            Print("CONTINUE");
         }

         //Print("ResetIfRequested();");
         ResetIfRequested();

         //Print("if(CheckFinish())");
         if (CheckFinish())
            break;

         sleep(3000);
      }

      SetFinish();
      Print("end Run");
   }
   catch(cv::Exception & e) {
      string msg = string("Run: cv:Exception: ") + e.what();
      cerr << "LocalMapping: " << msg << endl;
      Print(msg);
   }
   catch (const exception & e)
   {
      string msg = string("Run: exception: ") + e.what();
      cerr << "LocalMapping: " << msg << endl;
      Print(msg);
   }
   catch (...)
   {
      string msg = string("Run: an exception was not caught");
      cerr << "LocalMapping: " << msg << endl;
      Print(msg);
   }

   bool LocalMapping::InsertKeyFrame(KeyFrame * pKF)
   {
      if (!SetNotPause(true))
         return false;

      InterruptBA();

      // If the mapping accepts keyframes, insert keyframe.
      // Otherwise send a signal to interrupt BA
      if (!AcceptKeyFrames())
      {
         if (!mbMonocular)
         {
            if (KeyframesInQueue() >= 3)
               return false;
         }
         else
            return false;
      }

      unique_lock<mutex> lock(mMutexNewKFs);
      mlNewKeyFrames.push_back(pKF);

      SetNotPause(false);

      return true;
   }


   bool LocalMapping::CheckNewKeyFrames()
   {
      unique_lock<mutex> lock(mMutexNewKFs);
      return(!mlNewKeyFrames.empty());
   }

   void LocalMapping::ProcessNewKeyFrame(MapChangeEvent & mapChanges)
   {
      Print("begin ProcessNewKeyFrame");
      {
         unique_lock<mutex> lock(mMutexNewKFs);
         mpCurrentKeyFrame = mlNewKeyFrames.front();
         mlNewKeyFrames.pop_front();
      }

      // TODO OK - move this to MapperServer::InsertKeyFrame ?
      // Compute Bags of Words structures
      //mpCurrentKeyFrame->ComputeBoW(mVocab);

      // Associate MapPoints to the new keyframe and update normal and descriptor
      const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

      for (size_t i = 0; i < vpMapPointMatches.size(); i++)
      {
         MapPoint * pMP = vpMapPointMatches[i];
         if (pMP)
         {
            if (!pMP->isBad())
            {
               if (!pMP->IsInKeyFrame(mpCurrentKeyFrame))
               {
                  //Print("MapPoint is not in KeyFrame");
                  pMP->AddObservation(mpCurrentKeyFrame, i);
                  pMP->UpdateNormalAndDepth();
                  pMP->ComputeDistinctiveDescriptors();
               }
               else // this can only happen for new stereo points inserted by the Tracking
               {
                  mlpRecentAddedMapPoints.push_back(pMP);
               }
               // TODO OK - add pMP to updated points
               mapChanges.updatedMapPoints.insert(pMP);
            }
         }
      }

      // Update links in the Covisibility Graph
      mpCurrentKeyFrame->UpdateConnections();
      // TODO OK - add to created keyframes
      mapChanges.updatedKeyFrames.insert(mpCurrentKeyFrame);

      // TODO OK - move this to MapperServer::InsertKeyFrame ?
      // Insert Keyframe in Map
      //mMap.AddKeyFrame(mpCurrentKeyFrame);
      Print("end ProcessNewKeyFrame");
   }

   void LocalMapping::MapPointCulling(MapChangeEvent & mapChanges)
   {
      Print("begin MapPointCulling");
      // Check Recent Added MapPoints
      list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
      const unsigned long int nCurrentKFid = mpCurrentKeyFrame->GetId();

      int nThObs;
      if (mbMonocular)
         nThObs = 2;
      else
         nThObs = 3;
      const int cnThObs = nThObs;

      while (lit != mlpRecentAddedMapPoints.end())
      {
         MapPoint * pMP = *lit;
         if (pMP->isBad())
         {
            // TODO OK - add to deleted points
            mapChanges.deletedMapPoints.insert(pMP->GetId());
            lit = mlpRecentAddedMapPoints.erase(lit);
            //Print(to_string(pMP->GetId()) + "=MapPointId erased 1");
         }
         else if (pMP->GetFoundRatio() < 0.25f)
         {
            pMP->SetBadFlag(&mMap); // TODO OK - add to deleted points
            mapChanges.deletedMapPoints.insert(pMP->GetId());
            lit = mlpRecentAddedMapPoints.erase(lit);
            //Print(to_string(pMP->GetId()) + "=MapPointId erased 2");
         }
         else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs)
         {
            pMP->SetBadFlag(&mMap); // TODO OK - add to deleted points
            mapChanges.deletedMapPoints.insert(pMP->GetId());
            lit = mlpRecentAddedMapPoints.erase(lit);
            //Print(to_string(pMP->GetId()) + "=MapPointId erased 3");
         }
         else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
         {
            // TODO OK - add to deleted points
            // this point was created by the current keyframe (stereo)
            // add it to deleted points so that the tracker (that created it) will delete it
            mapChanges.deletedMapPoints.insert((*lit)->GetId());
            //Print(to_string((*lit)->GetId()) + "=MapPointId erased 4");
            lit = mlpRecentAddedMapPoints.erase(lit);
         }
         else
         {
            lit++;
         }
      }
      Print("end MapPointCulling");
   }

   void LocalMapping::CreateNewMapPoints(MapChangeEvent & mapChanges)
   {
      Print("begin CreateNewMapPoints");
      // Retrieve neighbor keyframes in covisibility graph
      int nn = 10;
      if (mbMonocular)
         nn = 20;
      const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

      ORBmatcher matcher(0.6, false);

      Print("transformations");
      cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
      cv::Mat Rwc1 = Rcw1.t();
      cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
      cv::Mat Tcw1(3, 4, CV_32F);
      Rcw1.copyTo(Tcw1.colRange(0, 3));
      tcw1.copyTo(Tcw1.col(3));
      cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

      Print("frame calibration");
      const float &fx1 = mpCurrentKeyFrame->mFC.fx;
      const float &fy1 = mpCurrentKeyFrame->mFC.fy;
      const float &cx1 = mpCurrentKeyFrame->mFC.cx;
      const float &cy1 = mpCurrentKeyFrame->mFC.cy;
      const float &invfx1 = mpCurrentKeyFrame->mFC.invfx;
      const float &invfy1 = mpCurrentKeyFrame->mFC.invfy;

      const float ratioFactor = 1.5f*mpCurrentKeyFrame->scaleFactor;

      int nnew = 0;

      Print("for loop");
      // Search matches with epipolar restriction and triangulate
      for (size_t i = 0; i < vpNeighKFs.size(); i++)
      {
         if (i > 0 && CheckNewKeyFrames())
         {
            Print("end CreateNewMapPoints 1");
            return;
         }

         KeyFrame * pKF2 = vpNeighKFs[i];

         // Check first that baseline is not too short
         cv::Mat Ow2 = pKF2->GetCameraCenter();
         cv::Mat vBaseline = Ow2 - Ow1;
         const float baseline = cv::norm(vBaseline);

         if (!mbMonocular)
         {
            if (baseline < pKF2->mFC.bl)
               continue;
         }
         else
         {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < 0.01)
               continue;
         }

         // Compute Fundamental Matrix
         cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

         // Search matches that fullfil epipolar constraint
         vector<pair<size_t, size_t> > vMatchedIndices;
         matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

         cv::Mat Rcw2 = pKF2->GetRotation();
         cv::Mat Rwc2 = Rcw2.t();
         cv::Mat tcw2 = pKF2->GetTranslation();
         cv::Mat Tcw2(3, 4, CV_32F);
         Rcw2.copyTo(Tcw2.colRange(0, 3));
         tcw2.copyTo(Tcw2.col(3));

         const float &fx2 = pKF2->mFC.fx;
         const float &fy2 = pKF2->mFC.fy;
         const float &cx2 = pKF2->mFC.cx;
         const float &cy2 = pKF2->mFC.cy;
         const float &invfx2 = pKF2->mFC.invfx;
         const float &invfy2 = pKF2->mFC.invfy;

         // Triangulate each match
         const int nmatches = vMatchedIndices.size();
         for (int ikp = 0; ikp < nmatches; ikp++)
         {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->keysUn[idx1];
            const float kp1_ur = mpCurrentKeyFrame->right[idx1];
            bool bStereo1 = kp1_ur >= 0;

            const cv::KeyPoint &kp2 = pKF2->keysUn[idx2];
            const float kp2_ur = pKF2->right[idx2];
            bool bStereo2 = kp2_ur >= 0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1)*invfx1, (kp1.pt.y - cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2)*invfx2, (kp2.pt.y - cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1 * xn1;
            cv::Mat ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
               cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mFC.bl / 2, mpCurrentKeyFrame->depth[idx1]));
            else if (bStereo2)
               cosParallaxStereo2 = cos(2 * atan2(pKF2->mFC.bl / 2, pKF2->depth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

            cv::Mat x3D;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays < 0.9998))
            {
               // Linear Triangulation Method
               cv::Mat A(4, 4, CV_32F);
               A.row(0) = xn1.at<float>(0)*Tcw1.row(2) - Tcw1.row(0);
               A.row(1) = xn1.at<float>(1)*Tcw1.row(2) - Tcw1.row(1);
               A.row(2) = xn2.at<float>(0)*Tcw2.row(2) - Tcw2.row(0);
               A.row(3) = xn2.at<float>(1)*Tcw2.row(2) - Tcw2.row(1);

               cv::Mat w, u, vt;
               cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

               x3D = vt.row(3).t();

               if (x3D.at<float>(3) == 0)
                  continue;

               // Euclidean coordinates
               x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

            }
            else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
            {
               x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
            }
            else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
            {
               x3D = pKF2->UnprojectStereo(idx2);
            }
            else
               continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
            if (z1 <= 0)
               continue;

            float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
            if (z2 <= 0)
               continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->levelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1)
            {
               float u1 = fx1 * x1*invz1 + cx1;
               float v1 = fy1 * y1*invz1 + cy1;
               float errX1 = u1 - kp1.pt.x;
               float errY1 = v1 - kp1.pt.y;
               if ((errX1*errX1 + errY1 * errY1) > 5.991*sigmaSquare1)
                  continue;
            }
            else
            {
               float u1 = fx1 * x1*invz1 + cx1;
               float u1_r = u1 - mpCurrentKeyFrame->mFC.blfx * invz1;
               float v1 = fy1 * y1*invz1 + cy1;
               float errX1 = u1 - kp1.pt.x;
               float errY1 = v1 - kp1.pt.y;
               float errX1_r = u1_r - kp1_ur;
               if ((errX1*errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8*sigmaSquare1)
                  continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->levelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
            const float invz2 = 1.0 / z2;
            if (!bStereo2)
            {
               float u2 = fx2 * x2*invz2 + cx2;
               float v2 = fy2 * y2*invz2 + cy2;
               float errX2 = u2 - kp2.pt.x;
               float errY2 = v2 - kp2.pt.y;
               if ((errX2*errX2 + errY2 * errY2) > 5.991*sigmaSquare2)
                  continue;
            }
            else
            {
               float u2 = fx2 * x2*invz2 + cx2;
               float u2_r = u2 - mpCurrentKeyFrame->mFC.blfx * invz2;
               float v2 = fy2 * y2*invz2 + cy2;
               float errX2 = u2 - kp2.pt.x;
               float errY2 = v2 - kp2.pt.y;
               float errX2_r = u2_r - kp2_ur;
               if ((errX2*errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8*sigmaSquare2)
                  continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D - Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D - Ow2;
            float dist2 = cv::norm(normal2);

            if (dist1 == 0 || dist2 == 0)
               continue;

            const float ratioDist = dist2 / dist1;
            const float ratioOctave = mpCurrentKeyFrame->scaleFactors[kp1.octave] / pKF2->scaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if (ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
               continue;

            // Triangulation is succesfull
            MapPoint * pMP = new MapPoint(NewMapPointId(), x3D, mpCurrentKeyFrame);
            //stringstream ss; ss << pMP->GetId() << "=MapPointId " << x3D; Print(ss);

            pMP->AddObservation(mpCurrentKeyFrame, idx1);
            pMP->AddObservation(pKF2, idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
            pKF2->AddMapPoint(pMP, idx2);
            // TODO OK - add to updated keyframes
            mapChanges.updatedKeyFrames.insert(mpCurrentKeyFrame);
            mapChanges.updatedKeyFrames.insert(pKF2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mMap.AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
            // TODO OK - add to created points
            mapChanges.updatedMapPoints.insert(pMP);

            nnew++;
         }
      }
      Print("end CreateNewMapPoints 2");
   }

   void LocalMapping::SearchInNeighbors(MapChangeEvent & mapChanges)
   {
      Print("begin SearchInNeighbors");
      // Retrieve neighbor keyframes
      int nn = 10;
      if (mbMonocular)
         nn = 20;
      Print("1");
      const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
      vector<KeyFrame *> vpTargetKFs;
      Print("2");
      for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
      {
         KeyFrame * pKFi = *vit;
         if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->GetId())
            continue;
         vpTargetKFs.push_back(pKFi);
         pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->GetId();

         // Extend to some second neighbors
         const vector<KeyFrame *> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
         for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++)
         {
            KeyFrame * pKFi2 = *vit2;
            if (pKFi2->isBad()
               || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->GetId()
               || pKFi2->GetId() == mpCurrentKeyFrame->GetId())
               continue;
            vpTargetKFs.push_back(pKFi2);
         }
      }


      Print("3");
      // Search matches by projection from current KF in target KFs
      ORBmatcher matcher;
      vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
      Print("4");
      for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
      {
         KeyFrame * pKFi = *vit;

         matcher.Fuse(mapChanges, pKFi, vpMapPointMatches, &mMap);
      }

      Print("5");
      // Search matches by projection from target KFs in current KF
      vector<MapPoint *> vpFuseCandidates;
      vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

      Print("6");
      for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++)
      {
         KeyFrame * pKFi = *vitKF;

         vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

         for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++)
         {
            MapPoint * pMP = *vitMP;
            if (!pMP)
               continue;
            if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->GetId())
               continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->GetId();
            vpFuseCandidates.push_back(pMP);
         }
      }

      Print("7");
      matcher.Fuse(mapChanges, mpCurrentKeyFrame, vpFuseCandidates, &mMap);


      Print("8");
      // Update points
      // TODO OK - only update points that were touched during Fuse
      vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
      for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
      {
         MapPoint * pMP = vpMapPointMatches[i];
         if (pMP)
         {
            if (!pMP->isBad() && mapChanges.updatedMapPoints.count(pMP) == 1)
            {
               pMP->ComputeDistinctiveDescriptors();
               pMP->UpdateNormalAndDepth();
            }
         }
      }

      Print("9");
      // Update connections in covisibility graph
      mpCurrentKeyFrame->UpdateConnections();
      Print("end SearchInNeighbors");
   }

   cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
   {
      cv::Mat R1w = pKF1->GetRotation();
      cv::Mat t1w = pKF1->GetTranslation();
      cv::Mat R2w = pKF2->GetRotation();
      cv::Mat t2w = pKF2->GetTranslation();

      cv::Mat R12 = R1w * R2w.t();
      cv::Mat t12 = -R1w * R2w.t()*t2w + t1w;

      cv::Mat t12x = SkewSymmetricMatrix(t12);

      const cv::Mat &K1 = pKF1->mFC.K;
      const cv::Mat &K2 = pKF2->mFC.K;


      return K1.t().inv()*t12x*R12*K2.inv();
   }

   void LocalMapping::RequestPause()
   {
      Print("begin RequestPause");
      unique_lock<mutex> lock(mMutexPause);
      mbPauseRequested = true;
      NotifyPauseRequested(mbPauseRequested);
      unique_lock<mutex> lock2(mMutexNewKFs);
      mbAbortBA = true;
      Print("end RequestPause");
   }

   bool LocalMapping::Pause()
   {
      unique_lock<mutex> lock(mMutexPause);
      if (mbPauseRequested && !mbNotPause)
      {
         mbPaused = true;
         //Print("PAUSE");
         return true;
      }

      return false;
   }

   bool LocalMapping::IsPaused()
   {
      unique_lock<mutex> lock(mMutexPause);
      return mbPaused;
   }

   bool LocalMapping::PauseRequested()
   {
      unique_lock<mutex> lock(mMutexPause);
      return mbPauseRequested;
   }

   void LocalMapping::Resume()
   {
      unique_lock<mutex> lock(mMutexPause);
      unique_lock<mutex> lock2(mMutexFinish);
      if (mbFinished)
         return;
      mbPaused = false;
      mbPauseRequested = false;
      NotifyPauseRequested(mbPauseRequested);
      for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
         delete *lit;
      mlNewKeyFrames.clear();

      Print("RESUME");
   }

   bool LocalMapping::AcceptKeyFrames()
   {
      unique_lock<mutex> lock(mMutexAccept);
      return mbAcceptKeyFrames;
   }

   void LocalMapping::SetAcceptKeyFrames(bool flag)
   {
      unique_lock<mutex> lock(mMutexAccept);
      mbAcceptKeyFrames = flag;
      NotifyAcceptKeyFrames(flag);
   }

   bool LocalMapping::SetNotPause(bool flag)
   {
      unique_lock<mutex> lock(mMutexPause);

      if (flag && mbPaused)
         return false;

      mbNotPause = flag;

      return true;
   }

   void LocalMapping::InterruptBA()
   {
      mbAbortBA = true;
   }

   void LocalMapping::KeyFrameCulling(MapChangeEvent & mapChanges)
   {
      Print("begin KeyFrameCulling");
      // Check redundant keyframes (only local keyframes)
      // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
      // in at least other 3 keyframes (in the same or finer scale)
      // We only consider close stereo points
      vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

      for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; vit++)
      {
         KeyFrame * pKF = *vit;
         if (pKF->GetId() == 0)
            continue;
         const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

         int nObs = 3;
         const int thObs = nObs;
         int nRedundantObservations = 0;
         int nMPs = 0;
         for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
         {
            MapPoint * pMP = vpMapPoints[i];
            if (pMP)
            {
               if (!pMP->isBad())
               {
                  if (!mbMonocular)
                  {
                     if (pKF->depth[i] > pKF->mFC.thDepth || pKF->depth[i] < 0)
                        continue;
                  }

                  nMPs++;
                  if (pMP->Observations() > thObs)
                  {
                     const int &scaleLevel = pKF->keysUn[i].octave;
                     const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                     int nObs = 0;
                     for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                     {
                        KeyFrame * pKFi = mit->first;
                        if (pKFi == pKF)
                           continue;
                        const int &scaleLeveli = pKFi->keysUn[mit->second].octave;

                        if (scaleLeveli <= scaleLevel + 1)
                        {
                           nObs++;
                           if (nObs >= thObs)
                              break;
                        }
                     }
                     if (nObs >= thObs)
                     {
                        nRedundantObservations++;
                     }
                  }
               }
            }
         }

         if (nRedundantObservations > 0.9*nMPs)
         {
            if (pKF->SetBadFlag(&mMap, &mKeyFrameDB))
            {
               // TODO OK - add to deleted keyframes
               mapChanges.deletedKeyFrames.insert(pKF->GetId());
            }
         }
      }
      Print("end KeyFrameCulling");
   }

   cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
   {
      return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
         v.at<float>(2), 0, -v.at<float>(0),
         -v.at<float>(1), v.at<float>(0), 0);
   }

   void LocalMapping::RequestReset()
   {
      {
         unique_lock<mutex> lock(mMutexReset);
         mbResetRequested = true;
      }

      while (1)
      {
         {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested)
               break;
         }
         sleep(3000);
      }
   }

   void LocalMapping::ResetIfRequested()
   {
      unique_lock<mutex> lock(mMutexReset);
      if (mbResetRequested)
      {
         Print("RESET MAP");
         mlNewKeyFrames.clear();
         mlpRecentAddedMapPoints.clear();
         mbResetRequested = false;
      }
   }

   void LocalMapping::RequestFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinishRequested = true;
   }

   bool LocalMapping::CheckFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinishRequested;
   }

   void LocalMapping::SetFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinished = true;
      unique_lock<mutex> lock2(mMutexPause);
      mbPaused = true;
   }

   bool LocalMapping::isFinished()
   {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinished;
   }

   unsigned long LocalMapping::NewMapPointId()
   {
      unsigned long temp = mNextMapPointId;
      mNextMapPointId += mMapPointIdSpan;
      return temp;
   }

} //namespace ORB_SLAM
