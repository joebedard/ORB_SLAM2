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

#include "Viewer.h"
#include "Sleep.h"

#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

   Viewer::Viewer(FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, Mapper & mapper, bool embeddedFrameDrawer, bool embeddedVertical) :
      SyncPrint("Viewer: "),
      mpMapDrawer(pMapDrawer),
      mMapper(mapper),
      mEmbeddedFrameDrawers(embeddedFrameDrawer),
      mEmbeddedVertical(embeddedVertical),
      mbFinishRequested(false),
      mbFinished(false),
      mbStopped(true),
      mbStopRequested(false),
      mbResetting(false)
   {
      if (pFrameDrawer)
         mvFrameDrawers.push_back(pFrameDrawer);

      if (pTracking)
         mvTrackers.push_back(pTracking);

      InitWindowTitle();
   }

   Viewer::Viewer(vector<FrameDrawer *> vFrameDrawers, MapDrawer * pMapDrawer, vector<Tracking *> vTrackers, Mapper & mapper, bool embeddedFrameDrawers, bool embeddedVertical) :
      SyncPrint("Viewer: "),
      mvFrameDrawers(vFrameDrawers),
      mpMapDrawer(pMapDrawer),
      mvTrackers(vTrackers),
      mMapper(mapper),
      mEmbeddedFrameDrawers(embeddedFrameDrawers),
      mEmbeddedVertical(embeddedVertical),
      mbFinishRequested(false),
      mbFinished(false),
      mbStopped(true),
      mbStopRequested(false),
      mbResetting(false)
   {
      InitWindowTitle();
   }

   void Viewer::InitWindowTitle()
   {
      if (mvTrackers.empty())
         mWindowTitle.append("ORB-SLAM2-TEAM Server");
      else
         mWindowTitle.append("ORB-SLAM2-TEAM Viewer");
   }

   size_t Viewer::GetMatrixDataSize(cv::Mat & mat)
   {
      return mat.rows * mat.cols * mat.elemSize();
   }

   void Viewer::CopyMatrixDataToBuffer(cv::Mat & mat, char * pData)
   {
      if (mat.isContinuous())
      {
         const unsigned int data_size = mat.rows * mat.cols * mat.elemSize();
         memcpy(pData, mat.ptr(), data_size);
         pData += data_size;
      }
      else
      {
         const unsigned int row_size = mat.cols * mat.elemSize();
         for (int i = 0; i < mat.rows; i++) 
         {
            memcpy(pData, mat.ptr(i), row_size);
            pData += row_size;
         }
      }
   }

   void Viewer::Run() try
   {
      Print("begin Run");
      const int MENU_WIDTH = 160;
      const int MENU_HEIGHT = 330;
      const int MAPVIEW_WIDTH = 1280;
      const int MAPVIEW_HEIGHT = 960;
      const int WINDOW_WIDTH = 1540;
      const int WINDOW_HEIGHT = 820;
      mbFinished = false;
      mbStopped = false;

      pangolin::WindowInterface & wi = pangolin::CreateWindowAndBind(mWindowTitle, WINDOW_WIDTH, WINDOW_HEIGHT);
      
      GLint maxTextureBufferSize;
      glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxTextureBufferSize);
      //maxTextureBufferSize = 1226 * 370; // force mono_kiti to resize the frame

      // 3D Mouse handler requires depth testing to be enabled
      glEnable(GL_DEPTH_TEST);

      // Issue specific OpenGl we might need
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      pangolin::OpenGlRenderState * vMapState;

      float viewpointX = mpMapDrawer->GetViewpointX();
      float viewpointY = mpMapDrawer->GetViewpointY();
      float viewpointZ = mpMapDrawer->GetViewpointZ();
      float viewpointF = mpMapDrawer->GetViewpointF();

      // Define Camera Render Object (for view / scene browsing)
      vMapState = new pangolin::OpenGlRenderState(
         pangolin::ProjectionMatrix(MAPVIEW_WIDTH, MAPVIEW_HEIGHT, viewpointF, viewpointF, MAPVIEW_WIDTH / 2.0, MAPVIEW_HEIGHT / 2.0, 0.1, 1000),
         pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
      );

      // Add named OpenGL viewport to window and provide 3D Handler
      pangolin::View & vMapView = pangolin::Display("Map View")
         .SetBounds(0.0, 1.0, 0.0, 1.0)
         .SetAspect(-(double)MAPVIEW_WIDTH / (double)MAPVIEW_HEIGHT)
         .SetHandler(new pangolin::Handler3D(*vMapState));

      pangolin::View & menu = pangolin::CreatePanel("menu")
         .SetBounds(0.5, 1.0, pangolin::Attach::Pix(-MENU_WIDTH), 1.0)
         .SetLayout(pangolin::LayoutVertical);

      pangolin::Var<bool> * pMenuFollowCamera = NULL;
      if (mvTrackers.size() == 1)
         pMenuFollowCamera = new pangolin::Var<bool>("menu.Follow Camera", true, true);

      pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
      pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
      pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
      pangolin::Var<bool> menuReset("menu.Reset", false, false);
      pangolin::Var<unsigned long> menuQuantityMP("menu.Map Points", 0);
      pangolin::Var<unsigned long> menuQuantityKF("menu.Key Frames", 0);
      pangolin::Var<unsigned long> menuQuantityLoops("menu.Loops", 0);

      vector<pangolin::Var<bool> *> vMenuLocalizationModes;
      for (int i = 0; i < mvTrackers.size(); ++i)
      {
         string name("menu.Tracking Only "); name.append(to_string(i+1));
         auto menuLocalizationMode = new pangolin::Var<bool>(name, false, true);
         vMenuLocalizationModes.push_back(menuLocalizationMode);
      }

      pangolin::View * d_multiviewTrackers = NULL;
      if (mEmbeddedFrameDrawers)
      {
         if (mEmbeddedVertical)
         {
            d_multiviewTrackers = &pangolin::Display("multiviewTrackers")
               .SetLayout(pangolin::LayoutEqual)
               .SetBounds(0.0, 1.0, 0.0, 0.3);
         }
         else
         {
            d_multiviewTrackers = &pangolin::Display("multiviewTrackers")
               .SetLayout(pangolin::LayoutEqual)
               .SetBounds(0.67, 1.0, 0.0, pangolin::Attach::Pix(-MENU_WIDTH));
         }
      }

      vector<pangolin::View *> vImageViews;
      vector<pangolin::GlTexture *> vImageTextures;
      vector<string> vTrackerWindowNames;
      for (int i = 0; i < mvFrameDrawers.size(); ++i)
      {
         FrameDrawer * fd = mvFrameDrawers[i];
         string name("Tracker View "); name.append(to_string(i+1));
         if (mEmbeddedFrameDrawers)
         {
            cv::Mat im = fd->DrawFrame();
            if (maxTextureBufferSize < fd->GetFrameWidth() * fd->GetFrameHeight())
            {
               vImageTextures.push_back(new pangolin::GlTexture(fd->GetFrameWidth() / 2, fd->GetFrameHeight(), GL_RGB, true, 0, GL_BGR, GL_UNSIGNED_BYTE));
               char * pData = new char[GetMatrixDataSize(im) / 2];
               mvRenderBuffers.push_back(pData);
            }
            else
            {
               vImageTextures.push_back(new pangolin::GlTexture(fd->GetFrameWidth(), fd->GetFrameHeight(), GL_RGB, true, 0, GL_BGR, GL_UNSIGNED_BYTE));
               char * pData = new char[GetMatrixDataSize(im)];
               mvRenderBuffers.push_back(pData);
            }

            pangolin::View & d_img = pangolin::Display(name)
               .SetAspect((double)fd->GetFrameWidth() / (double)fd->GetFrameHeight())
               .SetLock(pangolin::LockLeft, pangolin::LockTop);
            
            d_multiviewTrackers->AddDisplay(d_img);
            vImageViews.push_back(&d_img);
         }
         else
         {
            vTrackerWindowNames.push_back(name);
            cv::namedWindow(name, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
         }
      }

      bool bLocalizationMode = false;

      // workaround for OpenGL errors caused by pangolin::FinishFrame
      pangolin::process::Resize(WINDOW_WIDTH, WINDOW_HEIGHT);

      // check for OpenGL errors in initialization
      CheckGlDieOnError();

      while (true)
      {
         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

         menuQuantityKF = mMapper.KeyFramesInMap();
         menuQuantityMP = mMapper.MapPointsInMap();
         menuQuantityLoops = mMapper.LoopsInMap();

         for (int i = 0; i < mvTrackers.size(); ++i)
         {
            if (vMenuLocalizationModes[i]->GuiChanged())
            {
               if (*vMenuLocalizationModes[i])
                  mvTrackers[i]->ActivateLocalizationMode();
               else
                  mvTrackers[i]->DeactivateLocalizationMode();
            }
         }

         // draw the map
         {
            if (pMenuFollowCamera && *pMenuFollowCamera)
            {
               mpMapDrawer->Follow(*vMapState);
            }

            vMapView.Activate(*vMapState);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glBegin(GL_LINES); // draw coordinate system origin
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.1f, 0.0f, 0.0f);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.1f, 0.0f);
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.1f);
            glColor3f(1.0f, 1.0f, 1.0f);
            glEnd();

            mpMapDrawer->DrawCurrentCameras();
            if (menuShowKeyFrames || menuShowGraph)
               mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
               mpMapDrawer->DrawMapPoints();
         }


         for (int i = 0; i < mvFrameDrawers.size(); ++i)
         {
            FrameDrawer * fd = mvFrameDrawers[i];
            cv::Mat im = fd->DrawFrame();
            if (mEmbeddedFrameDrawers)
            {
               cv::Mat flipped = cv::Mat(im.rows, im.cols, CV_8UC3, cv::Scalar(0, 0, 0));
               cv::flip(im, flipped, 0);
               char * pBuffer = mvRenderBuffers[i];
               if (maxTextureBufferSize < fd->GetFrameWidth() * fd->GetFrameHeight())
               {
                  cv::Mat squished = cv::Mat(flipped.rows, flipped.cols / 2, CV_8UC3, cv::Scalar(0, 0, 0));
                  cv::resize(flipped, squished, cv::Size(flipped.cols / 2, flipped.rows), cv::INTER_AREA);
                  CopyMatrixDataToBuffer(squished, pBuffer);
                  vImageTextures[i]->Upload((void *)pBuffer, GL_BGR, GL_UNSIGNED_BYTE);
               }
               else
               {
                  CopyMatrixDataToBuffer(flipped, pBuffer);
                  vImageTextures[i]->Upload((void *)pBuffer, GL_BGR, GL_UNSIGNED_BYTE);
               }
               vImageViews[i]->Activate();
               vImageTextures[i]->RenderToViewport();
            }
            else
            {
               cv::imshow(vTrackerWindowNames[i], im);
            }
         }

         pangolin::FinishFrame();
         //CheckGlDieOnError();
         if (!mEmbeddedFrameDrawers)
         {
            cv::waitKey(20);
         }
         else
         {
            sleep(20000);
         }

         if (pangolin::ShouldQuit())
         {
            RequestFinish();
         }

         if (menuReset && !mbResetting)
         {
            mbResetting = true;
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            for (int i = 0; i < mvTrackers.size(); ++i)
            {
               if (*vMenuLocalizationModes[i])
               {
                  mvTrackers[i]->DeactivateLocalizationMode();
                  *vMenuLocalizationModes[i] = false;
               }
            }
            if (pMenuFollowCamera)
               *pMenuFollowCamera = true;
            mMapper.Reset();
            mbResetting = false;
            menuReset = false;
         }

         if (Stop())
         {
            Print("while (IsStopped())");
            while (IsStopped())
            {
               sleep(3000);
            }
         }

         if (CheckFinish())
            break;
      }

      SetFinish();
      Print("end Run");
   }
   catch(cv::Exception & e) {
      string msg = string("RunViewer: cv::Exception: ") + e.what();
      cerr << "Viewer: " << msg << endl;
      Print(msg);
      SetFinish();
   }
   catch (const exception & e)
   {
      string msg = string("Run: exception: ") + e.what();
      cerr << "Viewer: " << msg << endl;
      Print(msg);
      SetFinish();
   }
   catch (...)
   {
      string msg = string("Run: an exception was not caught");
      cerr << "Viewer: " << msg << endl;
      Print(msg);
      SetFinish();
   }

   void Viewer::RequestFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinishRequested = true;
   }

   bool Viewer::CheckFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinishRequested;
   }

   void Viewer::SetFinish()
   {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinished = true;

      if (mEmbeddedFrameDrawers) {
         for (char * pBuffer : mvRenderBuffers)
            delete pBuffer;
      }
      else {
         cv::destroyAllWindows();
      }
      pangolin::Quit();
   }

   bool Viewer::IsFinished()
   {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinished;
   }

   void Viewer::RequestStop()
   {
      unique_lock<mutex> lock(mMutexStop);
      if (!mbStopped)
         mbStopRequested = true;
   }

   bool Viewer::IsStopped()
   {
      unique_lock<mutex> lock(mMutexStop);
      return mbStopped;
   }

   bool Viewer::Stop()
   {
      unique_lock<mutex> lock(mMutexStop);
      unique_lock<mutex> lock2(mMutexFinish);

      if (mbFinishRequested)
         return false;
      else if (mbStopRequested)
      {
         mbStopped = true;
         mbStopRequested = false;
         return true;
      }

      return false;

   }

   void Viewer::Resume()
   {
      unique_lock<mutex> lock(mMutexStop);
      mbStopped = false;
   }

}
