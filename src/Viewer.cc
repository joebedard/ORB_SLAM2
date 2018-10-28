/**
* This file is part of ORB-SLAM2-NET.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
* For more information see <https://github.com/joebedard/ORB_SLAM2_NET>
*
* ORB-SLAM2-NET is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2-NET is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2-NET. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include "Sleep.h"

#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

   Viewer::Viewer(FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, Mapper * pMapper, bool embeddedFrameDrawer) :
      SyncPrint("Viewer: ", false),
      mpMapper(pMapper),
      mEmbeddedFrameDrawers(embeddedFrameDrawer),
      mbFinishRequested(false),
      mbFinished(true),
      mbStopped(true),
      mbStopRequested(false),
      mbResetting(false),
      mWindowTitle("ORB-SLAM2-NET Viewer")
   {
      if (pFrameDrawer)
         mvFrameDrawers.push_back(pFrameDrawer);
      
      if (pMapDrawer)
         mvMapDrawers.push_back(pMapDrawer);

      if (pTracking)
         mvTrackers.push_back(pTracking);
   }

   Viewer::Viewer(vector<FrameDrawer *> vFrameDrawers, vector<MapDrawer *> vMapDrawers, vector<Tracking *> vTrackers, Mapper * pMapper, bool embeddedFrameDrawers) :
      SyncPrint("Viewer: "),
      mvFrameDrawers(vFrameDrawers),
      mvMapDrawers(vMapDrawers),
      mvTrackers(vTrackers),
      mpMapper(pMapper),
      mEmbeddedFrameDrawers(embeddedFrameDrawers),
      mbFinishRequested(false),
      mbFinished(true),
      mbStopped(true),
      mbStopRequested(false),
      mbResetting(false),
      mWindowTitle("ORB-SLAM2-NET Viewer")
   {
   }

   void Viewer::Run() try
   {
      const int MENU_WIDTH = 220;
      const int MAPVIEW_WIDTH = 1280;
      const int MAPVIEW_HEIGHT = 768;
      const int WINDOW_WIDTH = MENU_WIDTH + MAPVIEW_WIDTH;
      const int WINDOW_HEIGHT = MAPVIEW_HEIGHT;
      mbFinished = false;
      mbStopped = false;

      pangolin::CreateWindowAndBind(mWindowTitle, WINDOW_WIDTH, WINDOW_HEIGHT);

      GLint maxTextureBufferSize;
      glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxTextureBufferSize);
      //maxTextureBufferSize = 1241 * 376; // force mono_kiti to resize the frame

      // 3D Mouse handler requires depth testing to be enabled
      glEnable(GL_DEPTH_TEST);

      // Issue specific OpenGl we might need
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      pangolin::View & menu = pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(MENU_WIDTH));
      pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
      pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
      pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
      pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);

      vector<pangolin::Var<bool> *> vMenuLocalizationModes;
      for (int i = 0; i < mvTrackers.size(); ++i)
      {
         string name("menu.Localization Tracker "); name.append(to_string(i));
         auto menuLocalizationMode = new pangolin::Var<bool>(name, false, true);
         vMenuLocalizationModes.push_back(menuLocalizationMode);
      }

      pangolin::Var<bool> menuReset("menu.Reset", false, false);

      pangolin::View & d_multiviewMaps = pangolin::Display("multiviewMaps");
      pangolin::View & d_multiviewTrackers = pangolin::Display("multiviewTrackers");

      int mapviewWidth = MAPVIEW_WIDTH / mvMapDrawers.size();
      int mapviewHeight = MAPVIEW_HEIGHT;
      if (mEmbeddedFrameDrawers)
      {
         mapviewHeight /= 2;

         d_multiviewMaps.SetBounds(0.5, 1.0, pangolin::Attach::Pix(MENU_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqualHorizontal);

         d_multiviewTrackers.SetBounds(0.0, 0.5, pangolin::Attach::Pix(MENU_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqualHorizontal);
      }
      else
      {
         d_multiviewMaps.SetBounds(0.0, 1.0, pangolin::Attach::Pix(MENU_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqualHorizontal);

         // trackers view is empty. add it to the menu so it doesn't interfere with maps view
         menu.AddDisplay(d_multiviewTrackers);
      }

      vector<pangolin::OpenGlRenderState *> vMapStates;
      vector<pangolin::View *> vMapViews;
      for (int i = 0; i < mvMapDrawers.size(); ++i)
      {
         float viewpointX = mvMapDrawers[i]->GetViewpointX();
         float viewpointY = mvMapDrawers[i]->GetViewpointY();
         float viewpointZ = mvMapDrawers[i]->GetViewpointZ();
         float viewpointF = mvMapDrawers[i]->GetViewpointF();

         // Define Camera Render Object (for view / scene browsing)
         auto s_cam = new pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(mapviewWidth, mapviewHeight, viewpointF, viewpointF, mapviewWidth / 2.0, mapviewHeight / 2.0, 0.1, 1000),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
         );
         vMapStates.push_back(s_cam);

         // Add named OpenGL viewport to window and provide 3D Handler
         string name("Map View "); name.append(to_string(i));
         pangolin::View & d_cam = pangolin::Display(name)
            .SetAspect((double)mapviewWidth / (double)mapviewHeight)
            .SetHandler(new pangolin::Handler3D(*s_cam));
         vMapViews.push_back(&d_cam);

         d_multiviewMaps.AddDisplay(d_cam);
      }

      vector<pangolin::View *> vImageViews;
      vector<pangolin::GlTexture *> vImageTextures;
      vector<string> vTrackerWindowNames;
      for (int i = 0; i < mvFrameDrawers.size(); ++i)
      {
         FrameDrawer * fd = mvFrameDrawers[i];
         string name("Tracker View "); name.append(to_string(i));
         if (mEmbeddedFrameDrawers)
         {
            if (maxTextureBufferSize < fd->GetFrameWidth() * fd->GetFrameHeight())
            {
               vImageTextures.push_back(new pangolin::GlTexture(fd->GetFrameWidth() / 2, fd->GetFrameHeight(), GL_RGB, true, 0, GL_BGR, GL_UNSIGNED_BYTE));
            }
            else
            {
               vImageTextures.push_back(new pangolin::GlTexture(fd->GetFrameWidth(), fd->GetFrameHeight(), GL_RGB, true, 0, GL_BGR, GL_UNSIGNED_BYTE));
            }
            pangolin::View& d_img = pangolin::Display(name)
               .SetAspect((double)fd->GetFrameWidth() / (double)fd->GetFrameHeight());
            d_multiviewTrackers.AddDisplay(d_img);
            vImageViews.push_back(&d_img);
         }
         else
         {
            vTrackerWindowNames.push_back(name);
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

         for (int i = 0; i < mvMapDrawers.size(); ++i)
         {
            if (menuFollowCamera)
            {
               mvMapDrawers[i]->Follow(*vMapStates[i]);
            }

            vMapViews[i]->Activate(*vMapStates[i]);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glBegin(GL_LINES);
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

            mvMapDrawers[i]->DrawCurrentCameras();
            if (menuShowKeyFrames || menuShowGraph)
               mvMapDrawers[i]->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
               mvMapDrawers[i]->DrawMapPoints();
         }

         for (int i = 0; i < mvFrameDrawers.size(); ++i)
         {
            FrameDrawer * fd = mvFrameDrawers[i];
            cv::Mat im = fd->DrawFrame();
            if (mEmbeddedFrameDrawers)
            {
               Print("cv::Mat flipped = cv::Mat(im.rows, im.cols, CV_8UC3, cv::Scalar(0, 0, 0));");
               cv::Mat flipped = cv::Mat(im.rows, im.cols, CV_8UC3, cv::Scalar(0, 0, 0));
               cv::flip(im, flipped, 0);
               if (maxTextureBufferSize < fd->GetFrameWidth() * fd->GetFrameHeight())
               {
                  cv::Mat squished = cv::Mat(flipped.rows, flipped.cols / 2, CV_8UC3, cv::Scalar(0, 0, 0));
                  cv::resize(flipped, squished, cv::Size(flipped.cols / 2, flipped.rows), cv::INTER_AREA);
                  vImageTextures[i]->Upload((void *)squished.data, GL_BGR, GL_UNSIGNED_BYTE);
               }
               else
               {
                  vImageTextures[i]->Upload((void *)flipped.data, GL_BGR, GL_UNSIGNED_BYTE);
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
            cv::waitKey(1);
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
            menuFollowCamera = true;
            mpMapper->Reset();
            mbResetting = false;
            menuReset = false;
         }

         if (Stop())
         {
            while (isStopped())
            {
               sleep(3000);
            }
         }

         if (CheckFinish())
            break;
      }

      SetFinish();
      if (!mEmbeddedFrameDrawers)
      {
         cv::destroyAllWindows();
      }
      pangolin::Quit();
   }
   catch (const exception & e)
   {
      Print(string("Exception in Viewer thread: ") + e.what());
      SetFinish();
      if (!mEmbeddedFrameDrawers)
      {
         cv::destroyAllWindows();
      }
      pangolin::Quit();
   }
   catch (...)
   {
      Print("An exception was not caught in the Viewer thread.");
      SetFinish();
      if (!mEmbeddedFrameDrawers)
      {
         cv::destroyAllWindows();
      }
      pangolin::Quit();
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
   }

   bool Viewer::isFinished()
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

   bool Viewer::isStopped()
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

   void Viewer::Release()
   {
      unique_lock<mutex> lock(mMutexStop);
      mbStopped = false;
   }

}
