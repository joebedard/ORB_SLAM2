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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include<Eigen/StdVector>

#include "Converter.h"
#include "SyncPrint.h"

#include<mutex>

namespace ORB_SLAM2
{

   using namespace std;

   void Optimizer::Print(const char * message)
   {
      SyncPrint::Print("Optimizer: ", message);
   }

   void Optimizer::CreateGraphGlobalBundleAdjustment(
      Map & theMap,
      g2o::SparseOptimizer & optimizer,
      const bool bRobust,
      vector<KeyFrame*> & vpKFs,
      vector<MapPoint*> & vpMPs,
      vector<bool> & vbNotIncludedMP,
      id_type & maxKFid)
   {
      vpKFs = theMap.GetAllKeyFrames();
      vpMPs = theMap.GetAllMapPoints();
      vbNotIncludedMP.resize(vpMPs.size());

      // Set KeyFrame vertices
      for (size_t i = 0; i < vpKFs.size(); i++)
      {
         KeyFrame* pKF = vpKFs[i];
         if (pKF->isBad())
            continue;
         g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
         vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
         vSE3->setId(pKF->id);
         vSE3->setFixed(pKF->id == 0);
         if (!optimizer.addVertex(vSE3))
            Print("optimizer.addVertex(vSE3) failed");
         if (pKF->id > maxKFid)
            maxKFid = pKF->id;
      }

      const float thHuber2D = sqrt(5.99);
      const float thHuber3D = sqrt(7.815);

      // Set MapPoint vertices
      for (size_t i = 0; i < vpMPs.size(); i++)
      {
         MapPoint* pMP = vpMPs[i];
         if (pMP->isBad())
            continue;
         g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
         vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
         const id_type id = pMP->id + maxKFid + 1;
         if (id <= maxKFid)
            throw exception("Optimizer::GlobalBundleAdjustment: maximum id exceeded");
         vPoint->setId(id);
         vPoint->setMarginalized(true);
         if (!optimizer.addVertex(vPoint))
            Print("optimizer.addVertex(vPoint) failed");

         const map<KeyFrame*, size_t> observations = pMP->GetObservations();

         int nEdges = 0;
         //SET EDGES
         for (map<KeyFrame*, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
         {

            KeyFrame* pKF = mit->first;
            if (pKF->isBad() || pKF->id > maxKFid)
               continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->keysUn[mit->second];

            if (pKF->right[mit->second] < 0)
            {
               Eigen::Matrix<double, 2, 1> obs;
               obs << kpUn.pt.x, kpUn.pt.y;

               g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

               e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
               e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->id)));
               e->setMeasurement(obs);
               Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * pKF->invLevelSigma2[kpUn.octave];
               e->setInformation(info);

               if (bRobust)
               {
                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(thHuber2D);
               }

               e->fx = pKF->mFC.fx;
               e->fy = pKF->mFC.fy;
               e->cx = pKF->mFC.cx;
               e->cy = pKF->mFC.cy;

               if (!optimizer.addEdge(e))
                  Print("optimizer.addEdge(e) failed");
            }
            else
            {
               Eigen::Matrix<double, 3, 1> obs;
               const float kp_ur = pKF->right[mit->second];
               obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

               g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

               e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
               e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->id)));
               e->setMeasurement(obs);
               Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * pKF->invLevelSigma2[kpUn.octave];
               e->setInformation(info);

               if (bRobust)
               {
                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(thHuber3D); // TODO - swap with line above?
               }

               e->fx = pKF->mFC.fx;
               e->fy = pKF->mFC.fy;
               e->cx = pKF->mFC.cx;
               e->cy = pKF->mFC.cy;
               e->bf = pKF->mFC.blfx;

               if (!optimizer.addEdge(e))
                  Print("optimizer.addEdge(e) failed");
            }
         }

         if (nEdges == 0)
         {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
         }
         else
         {
            vbNotIncludedMP[i] = false;
         }
      }
   }

   void Optimizer::RecoverGraphGlobalBundleAdjustment(
      Map & theMap,
      g2o::SparseOptimizer & optimizer,
      const id_type loopKeyFrameId,
      vector<KeyFrame*> & vpKFs,
      vector<MapPoint*> & vpMPs,
      vector<bool> & vbNotIncludedMP,
      id_type & maxKFid)
   {
      /*
      For GlobalBundleAdjustment, graph recovery does not involve the 'edges' 
      (i.e. the links between KeyFrames and MapPoints). Only poses and positions
      of the KeyFrames and MapPoints, respectively, are updated. Thus, it is
      not necessary to lock on the map mutex.
      */

      //Keyframes
      for (size_t i = 0; i < vpKFs.size(); i++)
      {
         KeyFrame* pKF = vpKFs[i];
         if (pKF->isBad())
            continue;
         g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->id));
         g2o::SE3Quat SE3quat = vSE3->estimate();
         if (loopKeyFrameId == 0)
         {
            pKF->SetPose(Converter::toCvMat(SE3quat));
         }
         else
         {
            // this keyframe might be added to the map changes by the outer function
            pKF->mTcwGBA.create(4, 4, CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = loopKeyFrameId;
         }
      }

      //Points
      for (size_t i = 0; i < vpMPs.size(); i++)
      {
         if (vbNotIncludedMP[i])
            continue;

         MapPoint* pMP = vpMPs[i];

         if (pMP->isBad())
            continue;
         g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->id + maxKFid + 1));

         if (loopKeyFrameId == 0)
         {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
         }
         else
         {
            // this mappoint might be added to map changes by outer function
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = loopKeyFrameId;
         }
      }
   }

   void Optimizer::GlobalBundleAdjustment(
      Map & theMap,
      int nIterations, 
      bool * pbStopFlag, 
      const id_type loopKeyFrameId, 
      const bool bRobust)
   {
      Print("begin GlobalBundleAdjustment");

      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

      linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      if (pbStopFlag)
         optimizer.setForceStopFlag(pbStopFlag);

      vector<KeyFrame*> vpKFs;
      vector<MapPoint*> vpMPs;
      vector<bool> vbNotIncludedMP;
      id_type maxKFid = 0;
      if (loopKeyFrameId == 0)
      {
         // GBA is called during Tracking initialization, and there is already a lock on the map
         CreateGraphGlobalBundleAdjustment(theMap, optimizer, bRobust, vpKFs, vpMPs, vbNotIncludedMP, maxKFid);
      }
      else
      {
         // GBA is called from LoopClosing, we should lock the map
         Print("waiting to lock map");
         unique_lock<mutex> lock(theMap.mutexMapUpdate);
         Print("map is locked");

         CreateGraphGlobalBundleAdjustment(theMap, optimizer, bRobust, vpKFs, vpMPs, vbNotIncludedMP, maxKFid);
      }

      // Optimize!
      if (optimizer.initializeOptimization())
         optimizer.optimize(nIterations);
      else
         throw exception("optimizer.initializeOptimization() failed");

      // Recover optimized data
      RecoverGraphGlobalBundleAdjustment(theMap, optimizer, loopKeyFrameId, vpKFs, vpMPs, vbNotIncludedMP, maxKFid);

      Print("end GlobalBundleAdjustment");
   }

   int Optimizer::PoseOptimization(Frame *pFrame)
   {
      Print("begin PoseOptimization");

      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      int nInitialCorrespondences = 0;

      // Set Frame vertex
      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
      vSE3->setId(0);
      vSE3->setFixed(false);
      if (!optimizer.addVertex(vSE3))
         Print("optimizer.addVertex(vSE3) failed");

      // Set MapPoint vertices
      const int N = pFrame->N;

      vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
      vector<size_t> vnIndexEdgeMono;
      vpEdgesMono.reserve(N);
      vnIndexEdgeMono.reserve(N);

      vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
      vector<size_t> vnIndexEdgeStereo;
      vpEdgesStereo.reserve(N);
      vnIndexEdgeStereo.reserve(N);

      const float deltaMono = sqrt(5.991);
      const float deltaStereo = sqrt(7.815);


      {
         unique_lock<mutex> lock(MapPoint::mGlobalMutex);

         for (int i = 0; i < N; i++)
         {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP)
            {
               // Monocular observation
               if (pFrame->mvuRight[i] < 0)
               {
                  nInitialCorrespondences++;
                  pFrame->mvbOutlier[i] = false;

                  Eigen::Matrix<double, 2, 1> obs;
                  const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                  obs << kpUn.pt.x, kpUn.pt.y;

                  g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                  e->setMeasurement(obs);
                  const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                  e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(deltaMono);

                  e->fx = pFrame->mFC->fx;
                  e->fy = pFrame->mFC->fy;
                  e->cx = pFrame->mFC->cx;
                  e->cy = pFrame->mFC->cy;
                  cv::Mat Xw = pMP->GetWorldPos();
                  e->Xw[0] = Xw.at<float>(0);
                  e->Xw[1] = Xw.at<float>(1);
                  e->Xw[2] = Xw.at<float>(2);

                  optimizer.addEdge(e);

                  vpEdgesMono.push_back(e);
                  vnIndexEdgeMono.push_back(i);
               }
               else  // Stereo observation
               {
                  nInitialCorrespondences++;
                  pFrame->mvbOutlier[i] = false;

                  //SET EDGE
                  Eigen::Matrix<double, 3, 1> obs;
                  const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                  const float &kp_ur = pFrame->mvuRight[i];
                  obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                  g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                  e->setMeasurement(obs);
                  const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                  Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                  e->setInformation(Info);

                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(deltaStereo);

                  e->fx = pFrame->mFC->fx;
                  e->fy = pFrame->mFC->fy;
                  e->cx = pFrame->mFC->cx;
                  e->cy = pFrame->mFC->cy;
                  e->bf = pFrame->mFC->blfx;
                  cv::Mat Xw = pMP->GetWorldPos();
                  e->Xw[0] = Xw.at<float>(0);
                  e->Xw[1] = Xw.at<float>(1);
                  e->Xw[2] = Xw.at<float>(2);

                  optimizer.addEdge(e);

                  vpEdgesStereo.push_back(e);
                  vnIndexEdgeStereo.push_back(i);
               }
            }

         }
      }


      if (nInitialCorrespondences < 3)
      {
         Print("end PoseOptimization 1");
         return 0;
      }

      // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
      // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
      const float chi2Mono[4] = { 5.991f, 5.991f, 5.991f , 5.991f };
      const float chi2Stereo[4] = { 7.815f, 7.815f, 7.815f, 7.815f };
      const int its[4] = { 10,10,10,10 };

      int nBad = 0;
      for (size_t it = 0; it < 4; it++)
      {

         vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
         optimizer.initializeOptimization(0);
         optimizer.optimize(its[it]);

         nBad = 0;
         for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
         {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx])
            {
               e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
               pFrame->mvbOutlier[idx] = true;
               e->setLevel(1);
               nBad++;
            }
            else
            {
               pFrame->mvbOutlier[idx] = false;
               e->setLevel(0);
            }

            if (it == 2)
               e->setRobustKernel(0);
         }

         for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
         {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx])
            {
               e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it])
            {
               pFrame->mvbOutlier[idx] = true;
               e->setLevel(1);
               nBad++;
            }
            else
            {
               e->setLevel(0);
               pFrame->mvbOutlier[idx] = false;
            }

            if (it == 2)
               e->setRobustKernel(0);
         }

         if (optimizer.edges().size() < 10)
            break;
      }

      // Recover optimized pose and return number of inliers
      g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
      g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
      cv::Mat pose = Converter::toCvMat(SE3quat_recov);
      pFrame->SetPose(pose);

      Print("end PoseOptimization 2");
      return nInitialCorrespondences - nBad;
   }

   void Optimizer::CreateGraphLocalBundleAdjustment(
      std::mutex & mutexMapUpdate,
      KeyFrame * pKF,
      g2o::SparseOptimizer & optimizer,
      id_type & maxKFid,
      list<KeyFrame*> & lLocalKeyFrames,
      list<MapPoint*> & lLocalMapPoints,
      vector<g2o::EdgeSE3ProjectXYZ*> & vpEdgesMono,
      vector<KeyFrame*> & vpEdgeKFMono,
      vector<MapPoint*> & vpMapPointEdgeMono,
      vector<g2o::EdgeStereoSE3ProjectXYZ*> & vpEdgesStereo,
      vector<KeyFrame*> & vpEdgeKFStereo,
      vector<MapPoint*> & vpMapPointEdgeStereo)
   {
      Print("waiting to lock map");
      unique_lock<mutex> lock(mutexMapUpdate);
      Print("map is locked");

      lLocalKeyFrames.push_back(pKF);
      pKF->mnBALocalForKF = pKF->id;

      // Local KeyFrames: First Breath Search from Current Keyframe
      const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
      for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
      {
         KeyFrame* pKFi = vNeighKFs[i];
         pKFi->mnBALocalForKF = pKF->id;
         if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
      }

      // Local MapPoints seen in Local KeyFrames
      for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
      {
         vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
         for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
         {
            MapPoint* pMP = *vit;
            if (pMP)
               if (!pMP->isBad())
                  if (pMP->mnBALocalForKF != pKF->id)
                  {
                     lLocalMapPoints.push_back(pMP);
                     pMP->mnBALocalForKF = pKF->id;
                  }
         }
      }

      // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
      list<KeyFrame*> lFixedCameras;
      for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
      {
         std::map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
         for (std::map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
         {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->id && pKFi->mnBAFixedForKF != pKF->id)
            {
               pKFi->mnBAFixedForKF = pKF->id;
               if (!pKFi->isBad())
                  lFixedCameras.push_back(pKFi);
            }
         }
      }

      // Set Local KeyFrame vertices
      for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
      {
         KeyFrame* pKFi = *lit;
         g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
         vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
         vSE3->setId(pKFi->id);
         vSE3->setFixed(pKFi->id == 0);
         if (!optimizer.addVertex(vSE3))
            Print("optimizer.addVertex(vSE3)) failed");
         if (pKFi->id > maxKFid)
            maxKFid = pKFi->id;
      }

      // Set Fixed KeyFrame vertices
      for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
      {
         KeyFrame* pKFi = *lit;
         g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
         vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
         vSE3->setId(pKFi->id);
         vSE3->setFixed(true);
         if (!optimizer.addVertex(vSE3))
            Print("optimizer.addVertex(vSE3) failed");
         if (pKFi->id > maxKFid)
            maxKFid = pKFi->id;
      }

      // Set MapPoint vertices
      const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size())*lLocalMapPoints.size();

      vpEdgesMono.reserve(nExpectedSize);
      vpEdgeKFMono.reserve(nExpectedSize);
      vpMapPointEdgeMono.reserve(nExpectedSize);
      vpEdgesStereo.reserve(nExpectedSize);
      vpEdgeKFStereo.reserve(nExpectedSize);
      vpMapPointEdgeStereo.reserve(nExpectedSize);

      const float thHuberMono = sqrt(5.991);
      const float thHuberStereo = sqrt(7.815);

      for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
      {
         MapPoint* pMP = *lit;
         g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
         vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
         id_type id = pMP->id + maxKFid + 1;
         if (id < maxKFid)
            throw exception("Optimizer::LocalBundleAdjustment: maximum id exceeded");
         vPoint->setId(id);
         vPoint->setMarginalized(true);
         if (!optimizer.addVertex(vPoint))
            Print("optimizer.addVertex(vPoint) failed");

         const std::map<KeyFrame*, size_t> observations = pMP->GetObservations();

         //Set edges
         for (std::map<KeyFrame*, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
         {
            KeyFrame* pKFi = mit->first;

            if (!pKFi->isBad())
            {
               const cv::KeyPoint &kpUn = pKFi->keysUn[mit->second];

               // Monocular observation
               if (pKFi->right[mit->second] < 0)
               {
                  Eigen::Matrix<double, 2, 1> obs;
                  obs << kpUn.pt.x, kpUn.pt.y;

                  g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                  e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->id)));
                  e->setMeasurement(obs);
                  const float &invSigma2 = pKFi->invLevelSigma2[kpUn.octave];
                  e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(thHuberMono);

                  e->fx = pKFi->mFC.fx;
                  e->fy = pKFi->mFC.fy;
                  e->cx = pKFi->mFC.cx;
                  e->cy = pKFi->mFC.cy;

                  optimizer.addEdge(e);
                  vpEdgesMono.push_back(e);
                  vpEdgeKFMono.push_back(pKFi);
                  vpMapPointEdgeMono.push_back(pMP);
               }
               else // Stereo observation
               {
                  Eigen::Matrix<double, 3, 1> obs;
                  const float kp_ur = pKFi->right[mit->second];
                  obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                  g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                  e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->id)));
                  e->setMeasurement(obs);
                  const float &invSigma2 = pKFi->invLevelSigma2[kpUn.octave];
                  Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                  e->setInformation(Info);

                  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                  e->setRobustKernel(rk);
                  rk->setDelta(thHuberStereo);

                  e->fx = pKFi->mFC.fx;
                  e->fy = pKFi->mFC.fy;
                  e->cx = pKFi->mFC.cx;
                  e->cy = pKFi->mFC.cy;
                  e->bf = pKFi->mFC.blfx;

                  optimizer.addEdge(e);
                  vpEdgesStereo.push_back(e);
                  vpEdgeKFStereo.push_back(pKFi);
                  vpMapPointEdgeStereo.push_back(pMP);
               }
            }
         }
      }
   }

   void Optimizer::CheckGraphLocalBundleAdjustment(
      std::mutex & mutexMapUpdate,
      vector<g2o::EdgeSE3ProjectXYZ*> & vpEdgesMono,
      vector<MapPoint*> & vpMapPointEdgeMono,
      vector<g2o::EdgeStereoSE3ProjectXYZ*> & vpEdgesStereo,
      vector<MapPoint*> & vpMapPointEdgeStereo)
   {
      Print("waiting to lock map");
      unique_lock<mutex> lock(mutexMapUpdate);
      Print("map is locked");

      // Check inlier observations
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend;i++)
      {
         g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
         MapPoint* pMP = vpMapPointEdgeMono[i];

         if (pMP->isBad())
            continue;

         if (e->chi2() > 5.991 || !e->isDepthPositive())
         {
            e->setLevel(1);
         }

         e->setRobustKernel(0);
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend;i++)
      {
         g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
         MapPoint* pMP = vpMapPointEdgeStereo[i];

         if (pMP->isBad())
            continue;

         if (e->chi2() > 7.815 || !e->isDepthPositive())
         {
            e->setLevel(1);
         }

         e->setRobustKernel(0);
      }
   }

   void Optimizer::RecoverGraphLocalBundleAdjustment(
      Map & theMap, 
      g2o::SparseOptimizer & optimizer,
      id_type & maxKFid,
      list<KeyFrame*> & lLocalKeyFrames,
      list<MapPoint*> & lLocalMapPoints,
      vector<g2o::EdgeSE3ProjectXYZ*> & vpEdgesMono,
      vector<KeyFrame*> & vpEdgeKFMono,
      vector<MapPoint*> & vpMapPointEdgeMono,
      vector<g2o::EdgeStereoSE3ProjectXYZ*> & vpEdgesStereo,
      vector<KeyFrame*> & vpEdgeKFStereo,
      vector<MapPoint*> & vpMapPointEdgeStereo)
   {
      vector<pair<KeyFrame*, MapPoint*> > vToErase;
      vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

      // Check inlier observations       
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend;i++)
      {
         g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
         MapPoint* pMP = vpMapPointEdgeMono[i];

         if (pMP->isBad())
            continue;

         if (e->chi2() > 5.991 || !e->isDepthPositive())
         {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
         }
      }

      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend;i++)
      {
         g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
         MapPoint* pMP = vpMapPointEdgeStereo[i];

         if (pMP->isBad())
            continue;

         if (e->chi2() > 7.815 || !e->isDepthPositive())
         {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
         }
      }

      // Get Map Mutex
      Print("waiting to lock map");
      unique_lock<mutex> lock(theMap.mutexMapUpdate);
      Print("map is locked");

      if (!vToErase.empty())
      {
         for (size_t i = 0;i < vToErase.size();i++)
         {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi, &theMap);
         }
      }

      // Recover optimized data

      //Keyframes
      for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
      {
         KeyFrame* pKF = *lit;
         g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->id));
         g2o::SE3Quat SE3quat = vSE3->estimate();
         pKF->SetPose(Converter::toCvMat(SE3quat));
      }

      //Points
      for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
      {
         MapPoint* pMP = *lit;
         g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->id + maxKFid + 1));
         pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
         pMP->UpdateNormalAndDepth();
      }
   }

   void Optimizer::LocalBundleAdjustment(
      KeyFrame * pKF,
      bool * pbStopFlag,
      Map & theMap)
   {
      Print("begin LocalBundleAdjustment");

      // Setup optimizer
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

      linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

      g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      if (pbStopFlag)
         optimizer.setForceStopFlag(pbStopFlag);

      id_type maxKFid = 0;
      list<KeyFrame*> lLocalKeyFrames;
      list<MapPoint*> lLocalMapPoints;
      vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
      vector<KeyFrame*> vpEdgeKFMono;
      vector<MapPoint*> vpMapPointEdgeMono;
      vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
      vector<KeyFrame*> vpEdgeKFStereo;
      vector<MapPoint*> vpMapPointEdgeStereo;

      CreateGraphLocalBundleAdjustment(theMap.mutexMapUpdate, pKF, optimizer, maxKFid, lLocalKeyFrames, lLocalMapPoints, 
         vpEdgesMono, vpEdgeKFMono, vpMapPointEdgeMono, vpEdgesStereo, vpEdgeKFStereo, vpMapPointEdgeStereo);

      if (pbStopFlag)
         if (*pbStopFlag)
         {
            Print("end LocalBundleAdjustment 1");
            return;
         }

      if (optimizer.initializeOptimization())
         optimizer.optimize(5);
      else
         throw exception("optimizer.initializeOptimization() failed");

      bool bDoMore = true;

      if (pbStopFlag)
         if (*pbStopFlag)
            bDoMore = false;

      if (bDoMore)
      {
         CheckGraphLocalBundleAdjustment(theMap.mutexMapUpdate, vpEdgesMono, vpMapPointEdgeMono, vpEdgesStereo, vpMapPointEdgeStereo);

         // Optimize again without the outliers
         if (optimizer.initializeOptimization())
            optimizer.optimize(10);
         else
            throw exception("optimizer.initializeOptimization() failed");

      }

      RecoverGraphLocalBundleAdjustment(theMap, optimizer,
         maxKFid, lLocalKeyFrames, lLocalMapPoints,
         vpEdgesMono, vpEdgeKFMono, vpMapPointEdgeMono,
         vpEdgesStereo, vpEdgeKFStereo, vpMapPointEdgeStereo);

      Print("end LocalBundleAdjustment 2");
   }

   void Optimizer::CreateGraphOptimize(
      KeyFrame * pCurKF,
      KeyFrame * pLoopKF, 
      mutex & mutexMapUpdate,
      g2o::SparseOptimizer & optimizer,
      const vector<KeyFrame *> & vpKFs, 
      const vector<MapPoint *> & vpMPs,
      const LoopClosing::KeyFrameAndPose & NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose & CorrectedSim3,
      const std::map<KeyFrame *, set<KeyFrame *> > & LoopConnections, 
      const bool & bFixScale,
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vScw,
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vCorrectedSwc,
      vector<g2o::VertexSim3Expmap*> & vpVertices)
   {
      Print("waiting to lock map");
      unique_lock<mutex> lock(mutexMapUpdate);
      Print("map is locked");

      const int minFeat = 100;

      // Set KeyFrame vertices
      for (size_t i = 0, iend = vpKFs.size(); i < iend;i++)
      {
         KeyFrame* pKF = vpKFs[i];
         if (pKF->isBad())
            continue;
         g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

         const id_type nIDi = pKF->id;

         LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

         if (it != CorrectedSim3.end())
         {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
         }
         else
         {
            Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw, tcw, 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
         }

         if (pKF == pLoopKF)
            VSim3->setFixed(true);

         VSim3->setId(nIDi);
         VSim3->setMarginalized(false);
         VSim3->_fix_scale = bFixScale;

         if (!optimizer.addVertex(VSim3))
            Print("optimizer.addVertex(VSim3) failed");

         vpVertices[nIDi] = VSim3;
      }


      set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

      const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

      // Set Loop edges
      for (std::map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
      {
         KeyFrame* pKF = mit->first;
         const long unsigned int nIDi = pKF->id;
         const set<KeyFrame*> &spConnections = mit->second;
         const g2o::Sim3 Siw = vScw[nIDi];
         const g2o::Sim3 Swi = Siw.inverse();

         for (set<KeyFrame*>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
         {
            const long unsigned int nIDj = (*sit)->id;
            if ((nIDi != pCurKF->id || nIDj != pLoopKF->id) && pKF->GetWeight(*sit) < minFeat)
               continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
         }
      }

      // Set normal edges
      for (KeyFrame * pKF : vpKFs)
      {
         const int nIDi = pKF->id;

         g2o::Sim3 Swi;

         LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

         if (iti != NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
         else
            Swi = vScw[nIDi].inverse();

         KeyFrame* pParentKF = pKF->GetParent();

         // Spanning tree edge
         if (pParentKF)
         {
            int nIDj = pParentKF->id;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if (itj != NonCorrectedSim3.end())
               Sjw = itj->second;
            else
               Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
         }

         // Loop edges
         const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
         for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
         {
            KeyFrame* pLKF = *sit;
            if (pLKF->id < pKF->id)
            {
               g2o::Sim3 Slw;

               LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

               if (itl != NonCorrectedSim3.end())
                  Slw = itl->second;
               else
                  Slw = vScw[pLKF->id];

               g2o::Sim3 Sli = Slw * Swi;
               g2o::EdgeSim3* el = new g2o::EdgeSim3();
               el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->id)));
               el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
               el->setMeasurement(Sli);
               el->information() = matLambda;
               optimizer.addEdge(el);
            }
         }

         // Covisibility graph edges
         const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
         for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
         {
            KeyFrame* pKFn = *vit;
            if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
               if (!pKFn->isBad() && pKFn->id < pKF->id)
               {
                  if (sInsertedEdges.count(make_pair(min(pKF->id, pKFn->id), max(pKF->id, pKFn->id))))
                     continue;

                  g2o::Sim3 Snw;

                  LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                  if (itn != NonCorrectedSim3.end())
                     Snw = itn->second;
                  else
                     Snw = vScw[pKFn->id];

                  g2o::Sim3 Sni = Snw * Swi;

                  g2o::EdgeSim3* en = new g2o::EdgeSim3();
                  en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->id)));
                  en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                  en->setMeasurement(Sni);
                  en->information() = matLambda;
                  optimizer.addEdge(en);
               }
            }
         }
      }
   }

   void Optimizer::RecoverGraphOptimize(
      KeyFrame * pCurKF,
      mutex & mutexMapUpdate,
      g2o::SparseOptimizer & optimizer,
      const vector<KeyFrame *> & vpKFs, 
      const vector<MapPoint *> & vpMPs,
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vScw,
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vCorrectedSwc)
   {
      Print("waiting to lock map");
      unique_lock<mutex> lock(mutexMapUpdate);
      Print("map is locked");

      // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
      for (size_t i = 0;i < vpKFs.size();i++)
      {
         KeyFrame* pKFi = vpKFs[i];

         const int nIDi = pKFi->id;

         g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
         g2o::Sim3 CorrectedSiw = VSim3->estimate();
         vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
         Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
         Eigen::Vector3d eigt = CorrectedSiw.translation();
         double s = CorrectedSiw.scale();

         eigt *= (1. / s); //[R t/s;0 1]

         cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

         pKFi->SetPose(Tiw);
      }

      // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
      for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
      {
         MapPoint* pMP = vpMPs[i];

         if (pMP->isBad())
            continue;

         int nIDr;
         if (pMP->mnCorrectedByKF == pCurKF->id)
         {
            nIDr = pMP->mnCorrectedReference;
         }
         else
         {
            KeyFrame * pRefKF = pMP->GetReferenceKeyFrame();
            if (pRefKF == NULL)
               throw exception("Optimizer::RecoverGraphOptimize detected a MapPoint without a reference KeyFrame");
            nIDr = pRefKF->id;
         }


         g2o::Sim3 Srw = vScw[nIDr];
         g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

         cv::Mat P3Dw = pMP->GetWorldPos();
         Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
         Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

         cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
         pMP->SetWorldPos(cvCorrectedP3Dw);

         pMP->UpdateNormalAndDepth();
      }
   }

   void Optimizer::OptimizeEssentialGraph(
      Map & theMap,
      KeyFrame * pLoopKF, 
      KeyFrame * pCurKF,
      const LoopClosing::KeyFrameAndPose & NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose & CorrectedSim3,
      const std::map<KeyFrame *, set<KeyFrame *> > & LoopConnections, 
      const bool & bFixScale)
   {
      Print("begin OptimizeEssentialGraph");

      // Setup optimizer
      g2o::SparseOptimizer optimizer;
      optimizer.setVerbose(false);
      g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
         new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
      g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

      solver->setUserLambdaInit(1e-16);
      optimizer.setAlgorithm(solver);

      const vector<KeyFrame*> vpKFs = theMap.GetAllKeyFrames();
      const vector<MapPoint*> vpMPs = theMap.GetAllMapPoints();

      const unsigned int nMaxKFid = theMap.GetMaxKFid();

      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);
      vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
      vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

      CreateGraphOptimize(
         pCurKF,
         pLoopKF,
         theMap.mutexMapUpdate,
         optimizer,
         vpKFs, 
         vpMPs,
         NonCorrectedSim3,
         CorrectedSim3,
         LoopConnections,
         bFixScale,
         vScw,
         vCorrectedSwc,
         vpVertices);

      // Optimize!
      if (optimizer.initializeOptimization())
         optimizer.optimize(20);
      else
         throw exception("optimizer.initializeOptimization() failed");

      RecoverGraphOptimize(
         pCurKF,
         theMap.mutexMapUpdate,
         optimizer,
         vpKFs, 
         vpMPs,
         vScw,
         vCorrectedSwc);

      Print("end OptimizeEssentialGraph");
   }

   int Optimizer::OptimizeSim3(
      KeyFrame * pKF1,
      KeyFrame * pKF2, 
      vector<MapPoint *> & vpMatches1, 
      g2o::Sim3 & g2oS12, 
      const float th2, 
      const bool bFixScale)
   {
      Print("begin OptimizeSim3");
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolverX::LinearSolverType * linearSolver;

      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

      g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      // Calibration
      const cv::Mat &K1 = pKF1->mFC.K;
      const cv::Mat &K2 = pKF2->mFC.K;

      // Camera poses
      const cv::Mat R1w = pKF1->GetRotation();
      const cv::Mat t1w = pKF1->GetTranslation();
      const cv::Mat R2w = pKF2->GetRotation();
      const cv::Mat t2w = pKF2->GetTranslation();

      // Set Sim3 vertex
      g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
      vSim3->_fix_scale = bFixScale;
      vSim3->setEstimate(g2oS12);
      vSim3->setId(0);
      vSim3->setFixed(false);
      vSim3->_principle_point1[0] = K1.at<float>(0, 2);
      vSim3->_principle_point1[1] = K1.at<float>(1, 2);
      vSim3->_focal_length1[0] = K1.at<float>(0, 0);
      vSim3->_focal_length1[1] = K1.at<float>(1, 1);
      vSim3->_principle_point2[0] = K2.at<float>(0, 2);
      vSim3->_principle_point2[1] = K2.at<float>(1, 2);
      vSim3->_focal_length2[0] = K2.at<float>(0, 0);
      vSim3->_focal_length2[1] = K2.at<float>(1, 1);
      if (!optimizer.addVertex(vSim3))
         Print("optimizer.addVertex failed(vSim3)");

      // Set MapPoint vertices
      const int N = vpMatches1.size();
      const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
      vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
      vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
      vector<size_t> vnIndexEdge;

      vnIndexEdge.reserve(2 * N);
      vpEdges12.reserve(2 * N);
      vpEdges21.reserve(2 * N);

      const float deltaHuber = sqrt(th2);

      int nCorrespondences = 0;

      for (int i = 0; i < N; i++)
      {
         if (!vpMatches1[i])
            continue;

         MapPoint* pMP1 = vpMapPoints1[i];
         MapPoint* pMP2 = vpMatches1[i];

         const id_type id1 = 2 * i + 1;
         const id_type id2 = 2 * (i + 1);

         const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

         if (pMP1 && pMP2)
         {
            if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
            {
               g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
               cv::Mat P3D1w = pMP1->GetWorldPos();
               cv::Mat P3D1c = R1w * P3D1w + t1w;
               vPoint1->setEstimate(Converter::toVector3d(P3D1c));
               vPoint1->setId(id1);
               vPoint1->setFixed(true);
               if (!optimizer.addVertex(vPoint1))
                  Print("optimizer.addVertex(vPoint1) failed");

               g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
               cv::Mat P3D2w = pMP2->GetWorldPos();
               cv::Mat P3D2c = R2w * P3D2w + t2w;
               vPoint2->setEstimate(Converter::toVector3d(P3D2c));
               vPoint2->setId(id2);
               vPoint2->setFixed(true);
               if (!optimizer.addVertex(vPoint2))
                  Print("optimizer.addVertex(vPoint2) failed");
            }
            else
               continue;
         }
         else
            continue;

         nCorrespondences++;

         // Set edge x1 = S12*X2
         Eigen::Matrix<double, 2, 1> obs1;
         const cv::KeyPoint &kpUn1 = pKF1->keysUn[i];
         obs1 << kpUn1.pt.x, kpUn1.pt.y;

         g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
         e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
         e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
         e12->setMeasurement(obs1);
         const float &invSigmaSquare1 = pKF1->invLevelSigma2[kpUn1.octave];
         e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

         g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
         e12->setRobustKernel(rk1);
         rk1->setDelta(deltaHuber);
         optimizer.addEdge(e12);

         // Set edge x2 = S21*X1
         Eigen::Matrix<double, 2, 1> obs2;
         const cv::KeyPoint &kpUn2 = pKF2->keysUn[i2];
         obs2 << kpUn2.pt.x, kpUn2.pt.y;

         g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

         e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
         e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
         e21->setMeasurement(obs2);
         float invSigmaSquare2 = pKF2->invLevelSigma2[kpUn2.octave];
         e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

         g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
         e21->setRobustKernel(rk2);
         rk2->setDelta(deltaHuber);
         optimizer.addEdge(e21);

         vpEdges12.push_back(e12);
         vpEdges21.push_back(e21);
         vnIndexEdge.push_back(i);
      }

      // Optimize!
      optimizer.initializeOptimization();
      optimizer.optimize(5);

      // Check inliers
      int nBad = 0;
      for (size_t i = 0; i < vpEdges12.size();i++)
      {
         g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
         g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
         if (!e12 || !e21)
            continue;

         if (e12->chi2() > th2 || e21->chi2() > th2)
         {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
         }
      }

      int nMoreIterations;
      if (nBad > 0)
         nMoreIterations = 10;
      else
         nMoreIterations = 5;

      if (nCorrespondences - nBad < 10)
      {
         Print("end OptimizeSim3 1");
         return 0;
      }

      // Optimize again only with inliers

      optimizer.initializeOptimization();
      optimizer.optimize(nMoreIterations);

      int nIn = 0;
      for (size_t i = 0; i < vpEdges12.size();i++)
      {
         g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
         g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
         if (!e12 || !e21)
            continue;

         if (e12->chi2() > th2 || e21->chi2() > th2)
         {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(NULL);
         }
         else
            nIn++;
      }

      // Recover optimized Sim3
      g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
      g2oS12 = vSim3_recov->estimate();

      Print("end OptimizeSim3 2");
      return nIn;
   }


} //namespace ORB_SLAM
