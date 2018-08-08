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
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind(mMapWindowTitle, 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View & menu = pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);

    vector<pangolin::Var<bool> *> vMenuLocalizationModes;
    for (int i = 0; i < mvTrackers.size(); ++i)
    {
       string name("menu.Localization Mode Tracker "); name.append(to_string(i));
       auto menuLocalizationMode = new pangolin::Var<bool>(name, false, true);
       vMenuLocalizationModes.push_back(menuLocalizationMode);
    }

    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuQuit("menu.Quit",false,false);

    pangolin::View & d_multi = pangolin::Display("multi")
       .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
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
            pangolin::ProjectionMatrix(1024, 768, viewpointF, viewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
        );
        vMapStates.push_back(s_cam);

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View & d_cam = pangolin::CreateDisplay()
           //.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
           .SetAspect(1024.0f / 768.0f)
           .SetHandler(new pangolin::Handler3D(*s_cam));
        vMapViews.push_back(&d_cam);

        d_multi.AddDisplay(d_cam);
    }

    vector<pangolin::View *> vImageViews;
    vector<pangolin::GlTexture *> vImageTextures;
    for (int i = 0; i < mvFrameDrawers.size(); ++i)
    {
       string name("img"); name.append(to_string(i));
       pangolin::View& d_img = pangolin::Display(name)
          .SetAspect(1024.0f / 768.0f);
       d_multi.AddDisplay(d_img);
       vImageViews.push_back(&d_img);
       FrameDrawer * fd = mvFrameDrawers[i];
       vImageTextures[i] = new pangolin::GlTexture(fd->GetWidth(), fd->GetHeight(), GL_LUMINANCE, false, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    }

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool bFollow = true;
    bool bLocalizationMode = false;

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

        pangolin::FinishFrame();
        
        for (int i = 0; i < mvFrameDrawers.size(); ++i)
        {
            cv::Mat im = mvFrameDrawers[i]->DrawFrame();
            vImageTextures[i]->Upload( im.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
            vImageViews[i]->Activate();
            vImageTextures[i]->RenderToViewport();
        }

        if (menuQuit)
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
