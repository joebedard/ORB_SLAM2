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

#include "MapDrawer.h"
#include <mutex>

namespace ORB_SLAM2
{


   MapDrawer::MapDrawer(cv::FileStorage & settings, Mapper & mapper) :
      SyncPrint("MapDrawer: ", false),
      mMapper(mapper),
      mMap(mapper.GetMap())
   {
      mKeyFrameSize = settings["Viewer.KeyFrameSize"];
      mKeyFrameLineWidth = settings["Viewer.KeyFrameLineWidth"];
      mGraphLineWidth = settings["Viewer.GraphLineWidth"];
      mPointSize = settings["Viewer.PointSize"];
      mCameraSize = settings["Viewer.CameraSize"];
      mCameraLineWidth = settings["Viewer.CameraLineWidth"];

      mViewpointX = settings["Viewer.ViewpointX"];
      mViewpointY = settings["Viewer.ViewpointY"];
      mViewpointZ = settings["Viewer.ViewpointZ"];
      mViewpointF = settings["Viewer.ViewpointF"];
   }

   void MapDrawer::Reset()
   {
      unique_lock<mutex> mutex(mMutexReferenceMapPoints);
      mvpReferenceMapPoints.clear();
   }

   void MapDrawer::Follow(pangolin::OpenGlRenderState & pRenderState)
   {
      unique_lock<mutex> lock(mMutexCamera);
      pangolin::OpenGlMatrix Twc;
      ConvertMatrixFromOpenCvToOpenGL(Twc, mCameraPose);
      pRenderState.Follow(Twc);
   }

   void MapDrawer::DrawMapPoints()
   {
      Print("begin DrawMapPoints");
      float currentColor[4];
      glGetFloatv(GL_CURRENT_COLOR, currentColor);

      const vector<MapPoint*> &vpMPs = mMap.GetAllMapPoints();
      if (vpMPs.empty())
      {
         Print("end DrawMapPoints 1");
         return;
      }

      set<MapPoint*> spRefMPs;
      {
         unique_lock<mutex> lock2(mMutexReferenceMapPoints);
         spRefMPs.insert(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end());
      }

      glPointSize(mPointSize);
      glBegin(GL_POINTS);
      glColor3f(0.0, 0.0, 0.0);

      for (MapPoint * pMP : vpMPs)
      {
         if (spRefMPs.count(pMP))
            continue;
         if (pMP->isBad())
            continue;
         cv::Mat pos = pMP->GetWorldPos();
         if (pos.empty())
         {
            stringstream ss;
            ss << pMP->GetId() << "=MapPointId GetWorldPos() is empty! replaced=";
            ss << pMP->GetReplaced() ? "true" : "false";
            //ss << mMap.GetReplacedMapPoint(pMP->GetId()) ? "true" : "false";
            Print(ss);
            //throw exception("GetWorldPos() is empty!!!!!!!!!!!!!!!!!!!!!!");
         }
         else
         {
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
         }
      }
      glEnd();

      glPointSize(mPointSize);
      glBegin(GL_POINTS);
      glColor3f(1.0, 0.0, 0.0);

      for (MapPoint * pMP : spRefMPs)
      {
         if (pMP->isBad())
            continue;
         cv::Mat pos = pMP->GetWorldPos();
         if (pos.empty())
         {
            //Print(to_string(pMP->GetId()) + "=MapPointId GetWorldPos() is empty! replaced=" + (pMP->GetReplaced() ? "true" : "false"));
            throw exception("GetWorldPos() is empty!!!!!!!!!!!!!!!!!!!!!!");
         }
         else
         {
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
         }
      }

      glEnd();
      glColor4fv(currentColor);
      Print("end DrawMapPoints 2");
   }

   void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
   {
      Print("begin DrawKeyFrames");
      float currentColor[4];
      glGetFloatv(GL_CURRENT_COLOR, currentColor);

      const float &w = mKeyFrameSize;
      const float h = w * 0.75;
      const float z = w * 0.6;

      const vector<KeyFrame*> vpKFs = mMap.GetAllKeyFrames();

      if (bDrawKF)
      {
         glLineWidth(mKeyFrameLineWidth);
         for (size_t i = 0; i < vpKFs.size(); i++)
         {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            if (pKF->isBad())
               glColor3f(0.0f, 0.0f, 0.2f);
            else
               glColor3f(0.0f, 0.0f, 1.0f);

            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();
         }
      }

      if (bDrawGraph)
      {
         glLineWidth(mGraphLineWidth);
         glBegin(GL_LINES);

         for (size_t i = 0; i < vpKFs.size(); i++)
         {
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if (!vCovKFs.empty())
            {
               for (vector<KeyFrame*>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
               {
                  if ((*vit)->GetId() < vpKFs[i]->GetId())
                     continue;
                  cv::Mat Ow2 = (*vit)->GetCameraCenter();
                  glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                  glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
               }
            }

            glColor4f(0.0f, 1.0f, 1.0f, 0.6f);
            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if (pParent)
            {
               cv::Mat Owp = pParent->GetCameraCenter();
               glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
               glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
            }

            glColor4f(1.0f, 1.0f, 0.0f, 0.6f);
            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for (set<KeyFrame*>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
            {
               if ((*sit)->GetId() < vpKFs[i]->GetId())
                  continue;
               cv::Mat Owl = (*sit)->GetCameraCenter();
               glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
               glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
            }
         }

         glEnd();
      }
      glColor4fv(currentColor);
      Print("end DrawKeyFrames");
   }

   void MapDrawer::DrawCurrentCameras()
   {
      Print("begin DrawCurrentCameras");
      float currentColor[4];
      glGetFloatv(GL_CURRENT_COLOR, currentColor);

      const float &w = mCameraSize;
      const float h = w * 0.75;
      const float z = w * 0.6;

      pangolin::OpenGlMatrix Twc, pivotGL;
      vector<cv::Mat> poses = mMapper.GetTrackerPoses();
      vector<cv::Mat> pivots = mMapper.GetTrackerPivots();
      assert(poses.size() == pivots.size());
      for (int i = 0; i < poses.size(); i++)
      {
         glPushMatrix();
         glLineWidth(mCameraLineWidth);

         // display the camera frustrum
         Twc.SetIdentity();
         ConvertMatrixFromOpenCvToOpenGL(Twc, poses[i]);
#ifdef HAVE_GLES
         glMultMatrixf(Twc.m);
#else
         glMultMatrixd(Twc.m);
#endif

         glColor3f(0.0f, 1.0f, 0.0f);
         glBegin(GL_LINES);
         glVertex3f(0, 0, 0);
         glVertex3f(w, h, z);
         glVertex3f(0, 0, 0);
         glVertex3f(w, -h, z);
         glVertex3f(0, 0, 0);
         glVertex3f(-w, -h, z);
         glVertex3f(0, 0, 0);
         glVertex3f(-w, h, z);

         glVertex3f(w, h, z);
         glVertex3f(w, -h, z);

         glVertex3f(-w, h, z);
         glVertex3f(-w, -h, z);

         glVertex3f(-w, h, z);
         glVertex3f(w, h, z);

         glVertex3f(-w, -h, z);
         glVertex3f(w, -h, z);

         glColor3f(1.0f, 0.0f, 0.0f);
         glVertex3f(0.0f, 0.0f, 0.0f);
         glVertex3f(0.1f, 0.0f, 0.0f);
         glColor3f(0.0f, 1.0f, 0.0f);
         glVertex3f(0.0f, 0.0f, 0.0f);
         glVertex3f(0.0f, 0.1f, 0.0f);
         glColor3f(0.0f, 0.0f, 1.0f);
         glVertex3f(0.0f, 0.0f, 0.0f);
         glVertex3f(0.0f, 0.0f, 0.1f);
         glEnd();

         // display a line for the pivot calibration
         pivotGL.SetIdentity();
         ConvertMatrixFromOpenCvToOpenGL(pivotGL, pivots[i]);
#ifdef HAVE_GLES
         glMultMatrixf(pivotGL.m);
#else
         glMultMatrixd(pivotGL.m);
#endif

         float x = pivots[i].at<float>(0, 3);
         float y = pivots[i].at<float>(1, 3);
         float z = pivots[i].at<float>(2, 3);

         glBegin(GL_LINES);
         glColor3f(1.0f, 0.0f, 1.0f);
         glVertex3f(0, 0, 0);
         glVertex3f(x, y, z);
         glEnd();

         glPopMatrix();
      }

      glColor4fv(currentColor);
      Print("end DrawCurrentCameras");
   }

   void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
   {
      unique_lock<mutex> lock(mMutexCamera);
      mCameraPose = Tcw.clone();
   }

   void MapDrawer::ConvertMatrixFromOpenCvToOpenGL(pangolin::OpenGlMatrix &M, cv::Mat pose)
   {
      //Print("begin ConvertMatrixFromOpenCvToOpenGL");
      if (!pose.empty())
      {
         cv::Mat Rwc(3, 3, CV_32F);
         cv::Mat twc(3, 1, CV_32F);
         {
            Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * pose.rowRange(0, 3).col(3);
         }

         M.m[0] = Rwc.at<float>(0, 0);
         M.m[1] = Rwc.at<float>(1, 0);
         M.m[2] = Rwc.at<float>(2, 0);
         M.m[3] = 0.0;

         M.m[4] = Rwc.at<float>(0, 1);
         M.m[5] = Rwc.at<float>(1, 1);
         M.m[6] = Rwc.at<float>(2, 1);
         M.m[7] = 0.0;

         M.m[8] = Rwc.at<float>(0, 2);
         M.m[9] = Rwc.at<float>(1, 2);
         M.m[10] = Rwc.at<float>(2, 2);
         M.m[11] = 0.0;

         M.m[12] = twc.at<float>(0);
         M.m[13] = twc.at<float>(1);
         M.m[14] = twc.at<float>(2);
         M.m[15] = 1.0;
      }
      else
         M.SetIdentity();
      //Print("end ConvertMatrixFromOpenCvToOpenGL");
   }

   void MapDrawer::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs)
   {
      //Print("begin SetReferenceMapPoints");
      unique_lock<mutex> lock(mMutexReferenceMapPoints);
      mvpReferenceMapPoints = vpMPs;
      //Print("end SetReferenceMapPoints");
   }

   float MapDrawer::GetViewpointX()
   {
      return mViewpointX;
   }

   float MapDrawer::GetViewpointY()
   {
      return mViewpointY;
   }

   float MapDrawer::GetViewpointZ()
   {
      return mViewpointZ;
   }

   float MapDrawer::GetViewpointF()
   {
      return mViewpointF;
   }

} //namespace ORB_SLAM
