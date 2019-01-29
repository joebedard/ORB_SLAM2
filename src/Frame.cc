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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2_TEAM
{

   long unsigned int Frame::nNextId = 0;

   Frame::Frame() {}

   //Copy Constructor
   Frame::Frame(const Frame & frame)
      : mpORBextractorLeft(frame.mpORBextractorLeft)
      , mpORBextractorRight(frame.mpORBextractorRight)
      , mTimeStamp(frame.mTimeStamp)
      , mFC(frame.mFC)
      , N(frame.N)
      , mvKeys(frame.mvKeys)
      , mvKeysRight(frame.mvKeysRight)
      , mvKeysUn(frame.mvKeysUn)
      , mvuRight(frame.mvuRight)
      , mvDepth(frame.mvDepth)
      , mBowVec(frame.mBowVec)
      , mFeatVec(frame.mFeatVec)
      , mDescriptors(frame.mDescriptors.clone())
      , mDescriptorsRight(frame.mDescriptorsRight.clone())
      , mvpMapPoints(frame.mvpMapPoints)
      , mvbOutlier(frame.mvbOutlier)
      , mnId(frame.mnId)
      , mpReferenceKF(frame.mpReferenceKF)
      , mnScaleLevels(frame.mnScaleLevels)
      , mfScaleFactor(frame.mfScaleFactor)
      , mfLogScaleFactor(frame.mfLogScaleFactor)
      , mvScaleFactors(frame.mvScaleFactors)
      , mvInvScaleFactors(frame.mvInvScaleFactors)
      , mvLevelSigma2(frame.mvLevelSigma2)
      , mvInvLevelSigma2(frame.mvInvLevelSigma2)
   {
      for (int i = 0;i < FRAME_GRID_COLS;i++)
         for (int j = 0; j < FRAME_GRID_ROWS; j++)
            mGrid[i][j] = frame.mGrid[i][j];

      if (!frame.mTcw.empty())
         SetPose(frame.mTcw);
   }


   Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, FrameCalibration * FC) 
      : mpORBextractorLeft(extractorLeft)
      , mpORBextractorRight(extractorRight)
      , mTimeStamp(timeStamp)
      , mFC(FC)
      , mpReferenceKF(static_cast<KeyFrame*>(NULL))
   {
      // Frame ID
      mnId = nNextId++;

      // Scale Level Info
      mnScaleLevels = mpORBextractorLeft->GetLevels();
      mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
      mfLogScaleFactor = log(mfScaleFactor);
      mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
      mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
      mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
      mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

      // ORB extraction
      thread threadLeft(&Frame::ExtractORBLeft, this, imLeft);
      thread threadRight(&Frame::ExtractORBRight, this, imRight);
      threadLeft.join();
      threadRight.join();

      N = mvKeys.size();

      if (mvKeys.empty())
         return;

      UndistortKeyPoints();

      ComputeStereoMatches();

      mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
      mvbOutlier = vector<bool>(N, false);

      AssignFeaturesToGrid();
   }

   Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor, FrameCalibration * FC)
      : mpORBextractorLeft(extractor)
      , mpORBextractorRight(static_cast<ORBextractor*>(NULL))
      , mTimeStamp(timeStamp)
      , mFC(FC)
   {
      // Frame ID
      mnId = nNextId++;

      // Scale Level Info
      mnScaleLevels = mpORBextractorLeft->GetLevels();
      mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
      mfLogScaleFactor = log(mfScaleFactor);
      mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
      mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
      mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
      mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

      // ORB extraction
      ExtractORBLeft(imGray);

      N = mvKeys.size();

      if (mvKeys.empty())
         return;

      UndistortKeyPoints();

      ComputeStereoFromRGBD(imDepth);

      mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
      mvbOutlier = vector<bool>(N, false);

      AssignFeaturesToGrid();
   }


   Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, FrameCalibration * FC)
      : mpORBextractorLeft(extractor)
      , mpORBextractorRight(static_cast<ORBextractor*>(NULL))
      , mTimeStamp(timeStamp)
      , mFC(FC)
   {
      // Frame ID
      mnId = nNextId++;

      // Scale Level Info
      mnScaleLevels = mpORBextractorLeft->GetLevels();
      mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
      mfLogScaleFactor = log(mfScaleFactor);
      mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
      mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
      mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
      mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

      // ORB extraction
      ExtractORBLeft(imGray);

      N = mvKeys.size();

      if (mvKeys.empty())
         return;

      UndistortKeyPoints();

      // Set no stereo information
      mvuRight = vector<float>(N, -1.0f);
      mvDepth = vector<float>(N, -1.0f);

      mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
      mvbOutlier = vector<bool>(N, false);

      AssignFeaturesToGrid();
   }

   void Frame::AssignFeaturesToGrid()
   {
      size_t nReserve = 0.5f*N / (size_t)(FRAME_GRID_COLS*FRAME_GRID_ROWS);
      for (unsigned int i = 0; i < FRAME_GRID_COLS;i++)
         for (unsigned int j = 0; j < FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

      for (size_t i = 0;i < N; i++)
      {
         const cv::KeyPoint &kp = mvKeysUn[i];

         int nGridPosX, nGridPosY;
         if (PosInGrid(kp, nGridPosX, nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
      }
   }

   void Frame::ExtractORBLeft(const cv::Mat &im)
   {
      mpORBextractorLeft->Extract(im, cv::Mat(), mvKeys, mDescriptors);
   }

   void Frame::ExtractORBRight(const cv::Mat &im)
   {
      mpORBextractorRight->Extract(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
   }

   void Frame::SetPose(cv::Mat Tcw)
   {
      mTcw = Tcw.clone();
      UpdatePoseMatrices();
   }

   void Frame::UpdatePoseMatrices()
   {
      mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
      mRwc = mRcw.t();
      mtcw = mTcw.rowRange(0, 3).col(3);
      mOw = -mRcw.t()*mtcw;
   }

   bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
   {
      pMP->mbTrackInView = false;

      // 3D in absolute coordinates
      cv::Mat P = pMP->GetWorldPos();

      // 3D in camera coordinates
      const cv::Mat Pc = mRcw * P + mtcw;
      const float &PcX = Pc.at<float>(0);
      const float &PcY = Pc.at<float>(1);
      const float &PcZ = Pc.at<float>(2);

      // Check positive depth
      if (PcZ < 0.0f)
         return false;

      // Project in image and check it is not outside
      const float invz = 1.0f / PcZ;
      const float u = mFC->fx*PcX*invz + mFC->cx;
      const float v = mFC->fy*PcY*invz + mFC->cy;

      if (u<mFC->minX || u>mFC->maxX)
         return false;
      if (v<mFC->minY || v>mFC->maxY)
         return false;

      // Check distance is in the scale invariance region of the MapPoint
      const float maxDistance = pMP->GetMaxDistanceInvariance();
      const float minDistance = pMP->GetMinDistanceInvariance();
      const cv::Mat PO = P - mOw;
      const float dist = cv::norm(PO);

      if (dist<minDistance || dist>maxDistance)
         return false;

      // Check viewing angle
      cv::Mat Pn = pMP->GetNormal();

      const float viewCos = PO.dot(Pn) / dist;

      if (viewCos < viewingCosLimit)
         return false;

      // Predict scale in the image
      const int nPredictedLevel = pMP->PredictScale(dist, this);

      // Data used by the tracking
      pMP->mbTrackInView = true;
      pMP->mTrackProjX = u;
      pMP->mTrackProjXR = u - mFC->blfx * invz;
      pMP->mTrackProjY = v;
      pMP->mnTrackScaleLevel = nPredictedLevel;
      pMP->mTrackViewCos = viewCos;

      return true;
   }

   vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
   {
      vector<size_t> vIndices;
      vIndices.reserve(N);

      const int nMinCellX = max(0, (int)floor((x - mFC->minX - r) * mFC->gridElementWidthInv));
      if (nMinCellX >= FRAME_GRID_COLS)
         return vIndices;

      const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mFC->minX + r) * mFC->gridElementWidthInv));
      if (nMaxCellX < 0)
         return vIndices;

      const int nMinCellY = max(0, (int)floor((y - mFC->minY - r) * mFC->gridElementHeightInv));
      if (nMinCellY >= FRAME_GRID_ROWS)
         return vIndices;

      const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mFC->minY + r) * mFC->gridElementHeightInv));
      if (nMaxCellY < 0)
         return vIndices;

      const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

      for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
      {
         for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
         {
            const vector<size_t> vCell = mGrid[ix][iy];
            if (vCell.empty())
               continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
               const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
               if (bCheckLevels)
               {
                  if (kpUn.octave < minLevel)
                     continue;
                  if (maxLevel >= 0)
                     if (kpUn.octave > maxLevel)
                        continue;
               }

               const float distx = kpUn.pt.x - x;
               const float disty = kpUn.pt.y - y;

               if (fabs(distx) < r && fabs(disty) < r)
                  vIndices.push_back(vCell[j]);
            }
         }
      }

      return vIndices;
   }

   bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
   {
      posX = round((kp.pt.x - mFC->minX) * mFC->gridElementWidthInv);
      posY = round((kp.pt.y - mFC->minY) * mFC->gridElementHeightInv);

      //Keypoint's coordinates are undistorted, which could cause to go out of the image
      if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
         return false;

      return true;
   }


   void Frame::ComputeBoW(ORBVocabulary & vocab)
   {
      if (mBowVec.empty())
      {
         vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
         vocab.transform(vCurrentDesc, mBowVec, mFeatVec, 4);
      }
   }

   void Frame::UndistortKeyPoints()
   {
      if (mFC->distCoef.at<float>(0) == 0.0)
      {
         mvKeysUn = mvKeys;
         return;
      }

      // Fill matrix with points
      cv::Mat mat(N, 2, CV_32F);
      for (size_t i = 0; i < N; i++)
      {
         mat.at<float>(i, 0) = mvKeys[i].pt.x;
         mat.at<float>(i, 1) = mvKeys[i].pt.y;
      }

      // Undistort points
      mat = mat.reshape(2);
      cv::undistortPoints(mat, mat, mFC->K, mFC->distCoef, cv::Mat(), mFC->K);
      mat = mat.reshape(1);

      // Fill undistorted keypoint vector
      mvKeysUn.resize(N);
      for (size_t i = 0; i < N; i++)
      {
         cv::KeyPoint kp = mvKeys[i];
         kp.pt.x = mat.at<float>(i, 0);
         kp.pt.y = mat.at<float>(i, 1);
         mvKeysUn[i] = kp;
      }
   }

   void Frame::ComputeStereoMatches()
   {
      mvuRight = vector<float>(N, -1.0f);
      mvDepth = vector<float>(N, -1.0f);

      const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

      const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

      //Assign keypoints to row table
      vector<vector<size_t> > vRowIndices(nRows, vector<size_t>());

      for (int i = 0; i < nRows; i++)
         vRowIndices[i].reserve(200);

      const int Nr = mvKeysRight.size();

      for (int iR = 0; iR < Nr; iR++)
      {
         const cv::KeyPoint &kp = mvKeysRight[iR];
         const float &kpY = kp.pt.y;
         const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
         const int maxr = ceil(kpY + r);
         const int minr = floor(kpY - r);

         for (int yi = minr;yi <= maxr;yi++)
            vRowIndices[yi].push_back(iR);
      }

      // Set limits for search
      const float minZ = mFC->bl;
      const float minD = 0;
      const float maxD = mFC->blfx / minZ;

      // For each left keypoint search a match in the right image
      vector<pair<int, int> > vDistIdx;
      vDistIdx.reserve(N);

      for (size_t iL = 0; iL < N; iL++)
      {
         const cv::KeyPoint &kpL = mvKeys[iL];
         const int &levelL = kpL.octave;
         const float &vL = kpL.pt.y;
         const float &uL = kpL.pt.x;

         const vector<size_t> &vCandidates = vRowIndices[vL];

         if (vCandidates.empty())
            continue;

         const float minU = uL - maxD;
         const float maxU = uL - minD;

         if (maxU < 0)
            continue;

         int bestDist = ORBmatcher::TH_HIGH;
         size_t bestIdxR = 0;

         const cv::Mat &dL = mDescriptors.row(iL);

         // Compare descriptor to right keypoints
         for (size_t iC = 0; iC < vCandidates.size(); iC++)
         {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if (kpR.octave<levelL - 1 || kpR.octave>levelL + 1)
               continue;

            const float &uR = kpR.pt.x;

            if (uR >= minU && uR <= maxU)
            {
               const cv::Mat &dR = mDescriptorsRight.row(iR);
               const int dist = ORBmatcher::DescriptorDistance(dL, dR);

               if (dist < bestDist)
               {
                  bestDist = dist;
                  bestIdxR = iR;
               }
            }
         }

         // Subpixel match by correlation
         if (bestDist < thOrbDist)
         {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
            IL.convertTo(IL, CV_32F);
            IL = IL - IL.at<float>(w, w) *cv::Mat::ones(IL.rows, IL.cols, CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
               continue;

            for (int incR = -L; incR <= +L; incR++)
            {
               cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
               IR.convertTo(IR, CV_32F);
               IR = IR - IR.at<float>(w, w) *cv::Mat::ones(IR.rows, IR.cols, CV_32F);

               float dist = cv::norm(IL, IR, cv::NORM_L1);
               if (dist < bestDist)
               {
                  bestDist = dist;
                  bestincR = incR;
               }

               vDists[L + incR] = dist;
            }

            if (bestincR == -L || bestincR == L)
               continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR = (dist1 - dist3) / (2.0f*(dist1 + dist3 - 2.0f*dist2));

            if (deltaR < -1 || deltaR>1)
               continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if (disparity >= minD && disparity < maxD)
            {
               if (disparity <= 0)
               {
                  disparity = 0.01;
                  bestuR = uL - 0.01;
               }
               mvDepth[iL] = mFC->blfx / disparity;
               mvuRight[iL] = bestuR;
               vDistIdx.push_back(pair<int, int>(bestDist, iL));
            }
         }
      }

      if (vDistIdx.size() <= 0)
         return;

      sort(vDistIdx.begin(), vDistIdx.end());
      const float median = vDistIdx[vDistIdx.size() / 2].first;
      const float thDist = 1.5f*1.4f*median;

      for (int i = vDistIdx.size() - 1;i >= 0;i--)
      {
         if (vDistIdx[i].first < thDist)
            break;
         else
         {
            mvuRight[vDistIdx[i].second] = -1;
            mvDepth[vDistIdx[i].second] = -1;
         }
      }
   }


   void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
   {
      mvuRight = vector<float>(N, -1);
      mvDepth = vector<float>(N, -1);

      for (size_t i = 0; i < N; i++)
      {
         const cv::KeyPoint &kp = mvKeys[i];
         const cv::KeyPoint &kpU = mvKeysUn[i];

         const float &v = kp.pt.y;
         const float &u = kp.pt.x;

         const float d = imDepth.at<float>(v, u);

         if (d > 0)
         {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x - mFC->blfx / d;
         }
      }
   }

   cv::Mat Frame::UnprojectStereo(const int &i)
   {
      const float z = mvDepth[i];
      if (z > 0)
      {
         const float u = mvKeysUn[i].pt.x;
         const float v = mvKeysUn[i].pt.y;
         const float x = (u - mFC->cx) * z * mFC->invfx;
         const float y = (v - mFC->cy) * z * mFC->invfy;
         cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
         return mRwc * x3Dc + mOw;
      }
      else
         return cv::Mat();
   }

} //namespace ORB_SLAM
