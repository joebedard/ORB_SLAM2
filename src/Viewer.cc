/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include "Viewer.h"
#include "Sleep.h"

#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking) :
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mbResetting(false)
{
   mvFrameDrawers.push_back(pFrameDrawer);
   mvMapDrawers.push_back(pMapDrawer);
   mvTrackers.push_back(pTracking);
   Initialize();
}

Viewer::Viewer(vector<FrameDrawer *> vFrameDrawers, vector<MapDrawer *> vMapDrawers, vector<Tracking *> vTrackers) :
   mvFrameDrawers(vFrameDrawers), mvMapDrawers(vMapDrawers), mvTrackers(vTrackers),
   mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mbResetting(false)
{
   Initialize();
}

void Viewer::Initialize()
{
   mMapWindowTitle.append("ORB-SLAM2 : Map Viewer");
}

void Viewer::Run()
{
    const int MENU_WIDTH = 220;
    const int MULTIVIEW_WIDTH = 1024;
    const int WINDOW_WIDTH = MENU_WIDTH + MULTIVIEW_WIDTH;
    const int WINDOW_HEIGHT = 768;
    const float MULTIVIEW_ASPECT = (float)MULTIVIEW_WIDTH / (float)WINDOW_HEIGHT;
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind(mMapWindowTitle, WINDOW_WIDTH, WINDOW_HEIGHT);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View & menu = pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(MENU_WIDTH));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);

    vector<pangolin::Var<bool> *> vMenuLocalizationModes;
    for (int i = 0; i < mvTrackers.size(); ++i)
    {
       string name("menu.Localization Tracker "); name.append(to_string(i));
       auto menuLocalizationMode = new pangolin::Var<bool>(name, false, true);
       vMenuLocalizationModes.push_back(menuLocalizationMode);
    }

    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::View & d_multi = pangolin::Display("multiview")
       .SetBounds(0.0, 1.0, pangolin::Attach::Pix(MENU_WIDTH), 1.0, MULTIVIEW_ASPECT)
       .SetLayout(pangolin::LayoutEqual);

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
            pangolin::ProjectionMatrix(MULTIVIEW_WIDTH, WINDOW_HEIGHT, viewpointF, viewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
        );
        vMapStates.push_back(s_cam);

        // Add named OpenGL viewport to window and provide 3D Handler
        string name("mapView"); name.append(to_string(i));
        pangolin::View & d_cam = pangolin::Display(name)
           //.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
           .SetAspect(MULTIVIEW_ASPECT)
           .SetHandler(new pangolin::Handler3D(*s_cam));
        vMapViews.push_back(&d_cam);

        d_multi.AddDisplay(d_cam);
    }

    vector<pangolin::View *> vImageViews;
    vector<pangolin::GlTexture *> vImageTextures;
    for (int i = 0; i < mvFrameDrawers.size(); ++i)
    {
       FrameDrawer * fd = mvFrameDrawers[i];
       vImageTextures.push_back(new pangolin::GlTexture(fd->GetWidth(), fd->GetHeight(), GL_RGB, true, 0, GL_BGR, GL_UNSIGNED_BYTE));
       string name("trackerView"); name.append(to_string(i));
       pangolin::View& d_img = pangolin::Display(name)
          //.SetAspect((float)fd->GetWidth() / (float)fd->GetHeight());
          .SetAspect(MULTIVIEW_ASPECT);
       d_multi.AddDisplay(d_img);
       vImageViews.push_back(&d_img);
    }

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool bFollow = true;
    bool bLocalizationMode = false;

    // workaround for OpenGL errors caused by pangolin::FinishFrame
    pangolin::process::Resize(WINDOW_WIDTH, WINDOW_HEIGHT); 

    // check for OpenGL errors in initialization
    CheckGlDieOnError();

    while(1)
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
           float viewpointX = mvMapDrawers[i]->GetViewpointX();
           float viewpointY = mvMapDrawers[i]->GetViewpointY();
           float viewpointZ = mvMapDrawers[i]->GetViewpointZ();
           float viewpointF = mvMapDrawers[i]->GetViewpointF();

           mvMapDrawers[i]->GetCurrentOpenGLCameraMatrix(Twc);

           if (menuFollowCamera && bFollow)
           {
              vMapStates[i]->Follow(Twc);
           }
           else if (menuFollowCamera && !bFollow)
           {
              vMapStates[i]->SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
              vMapStates[i]->Follow(Twc);
              bFollow = true;
           }
           else if (!menuFollowCamera && bFollow)
           {
              bFollow = false;
           }

           vMapViews[i]->Activate(*vMapStates[i]);
           glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
           mvMapDrawers[i]->DrawCurrentCamera(Twc);
           if (menuShowKeyFrames || menuShowGraph)
              mvMapDrawers[i]->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
           if (menuShowPoints)
              mvMapDrawers[i]->DrawMapPoints();
        }

        for (int i = 0; i < mvFrameDrawers.size(); ++i)
        {
            cv::Mat im = mvFrameDrawers[i]->DrawFrame();
            cv::Mat flipped = cv::Mat(im.rows, im.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::flip(im, flipped, 0);
            vImageTextures[i]->Upload( (void *)flipped.data, GL_BGR, GL_UNSIGNED_BYTE);
            vImageViews[i]->Activate();
            vImageTextures[i]->RenderToViewport();
        }

        pangolin::FinishFrame();
        //CheckGlDieOnError();

        if (pangolin::ShouldQuit())
        {
           RequestFinish();
        }

        if(menuReset && !mbResetting)
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
            bFollow = true;
            menuFollowCamera = true;
            for (auto t : mvTrackers)
            {
               t->Reset();
            }
            mbResetting = false;
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                sleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
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
    if(!mbStopped)
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

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
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
