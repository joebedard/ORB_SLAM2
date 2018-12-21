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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"

namespace ORB_SLAM2
{

   class LoopClosing;

   class Optimizer
   {
   public:

      static void GlobalBundleAdjustment(
         Map & theMap,
         MapChangeEvent & mapChanges,
         int nIterations = 5,
         bool * pbStopFlag = NULL,
         const id_type loopKeyFrameId = 0,
         const bool bRobust = true);

      static void LocalBundleAdjustment(
         KeyFrame * pKF,
         bool * pbStopFlag,
         Map & theMap,
         MapChangeEvent & mapChanges);

      static int PoseOptimization(Frame* pFrame);

      // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
      static void OptimizeEssentialGraph(
         Map & theMap,
         KeyFrame * pLoopKF,
         KeyFrame * pCurKF,
         const LoopClosing::KeyFrameAndPose & NonCorrectedSim3,
         const LoopClosing::KeyFrameAndPose & CorrectedSim3,
         const std::map<KeyFrame *, set<KeyFrame *> > & LoopConnections,
         const bool & bFixScale,
         MapChangeEvent & mapChanges);

      // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
      static int OptimizeSim3(
         KeyFrame * pKF1,
         KeyFrame * pKF2,
         std::vector<MapPoint *> & vpMatches1,
         g2o::Sim3 & g2oS12,
         const float th2,
         const bool bFixScale);

   private:

      static void Print(const char * message);

      static void CreateGraphLocalBundleAdjustment(
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
         vector<MapPoint*> & vpMapPointEdgeStereo);

      static void CheckGraphLocalBundleAdjustment(
         std::mutex & mutexMapUpdate,
         vector<g2o::EdgeSE3ProjectXYZ*> & vpEdgesMono,
         vector<MapPoint*> & vpMapPointEdgeMono,
         vector<g2o::EdgeStereoSE3ProjectXYZ*> & vpEdgesStereo,
         vector<MapPoint*> & vpMapPointEdgeStereo);

      static void RecoverGraphLocalBundleAdjustment(
         Map & theMap,
         MapChangeEvent & mapChanges,
         g2o::SparseOptimizer & optimizer,
         id_type & maxKFid,
         list<KeyFrame*> & lLocalKeyFrames,
         list<MapPoint*> & lLocalMapPoints,
         vector<g2o::EdgeSE3ProjectXYZ*> & vpEdgesMono,
         vector<KeyFrame*> & vpEdgeKFMono,
         vector<MapPoint*> & vpMapPointEdgeMono,
         vector<g2o::EdgeStereoSE3ProjectXYZ*> & vpEdgesStereo,
         vector<KeyFrame*> & vpEdgeKFStereo,
         vector<MapPoint*> & vpMapPointEdgeStereo);

      static void CreateGraphOptimize(
         KeyFrame * pCurKF,
         KeyFrame * pLoopKF, 
         mutex & mutexMapUpdate,
         MapChangeEvent & mapChanges, 
         g2o::SparseOptimizer & optimizer,
         const vector<KeyFrame *> & vpKFs, 
         const vector<MapPoint *> & vpMPs,
         const LoopClosing::KeyFrameAndPose & NonCorrectedSim3,
         const LoopClosing::KeyFrameAndPose & CorrectedSim3,
         const std::map<KeyFrame *, set<KeyFrame *> > & LoopConnections, 
         const bool & bFixScale,
         vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vScw,
         vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vCorrectedSwc,
         vector<g2o::VertexSim3Expmap*> & vpVertices);
         
      static void Optimizer::RecoverGraphOptimize(
         KeyFrame * pCurKF,
         mutex & mutexMapUpdate,
         MapChangeEvent & mapChanges, 
         g2o::SparseOptimizer & optimizer,
         const vector<KeyFrame *> & vpKFs, 
         const vector<MapPoint *> & vpMPs,
         vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vScw,
         vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > & vCorrectedSwc);
         
      static void CreateGraphGlobalBundleAdjustment(
         Map & theMap,
         g2o::SparseOptimizer & optimizer,
         const bool bRobust,
         vector<KeyFrame*> & vpKFs,
         vector<MapPoint*> & vpMPs,
         vector<bool> & vbNotIncludedMP,
         id_type & maxKFid);

      static void RecoverGraphGlobalBundleAdjustment(
         Map & theMap,
         MapChangeEvent & mapChanges,
         g2o::SparseOptimizer & optimizer,
         const id_type loopKeyFrameId,
         vector<KeyFrame*> & vpKFs,
         vector<MapPoint*> & vpMPs,
         vector<bool> & vbNotIncludedMP,
         id_type & maxKFid);
   };

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
