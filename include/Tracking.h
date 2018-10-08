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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "MapperServer.h"
#include "MapChangeEvent.h"
#include "MapObserver.h"
#include "Viewer.h"
#include "FrameDrawer.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Enums.h"
#include "SyncPrint.h"
#include "FrameCalibration.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;

class Tracking : SyncPrint
{  

public:
    Tracking(
        ORBVocabulary* pVoc, 
        FrameDrawer* pFrameDrawer, 
        MapDrawer* pMapDrawer,
        MapperServer* pMapper,
        cv::FileStorage & settings, 
        eSensor sensor
    );

    ~Tracking();

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetViewer(Viewer* pViewer);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();

    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();


public:

    eTrackingState mLastProcessedState;

    // Input sensor
    ORB_SLAM2::eSensor mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // if true, new keyframes are not sent to the mapper
    // if false, keyframes are sent to the mapper
    // in both cases, the map could still be updated by loop closure or bundle adjust
    bool mbOnlyTracking;

    // Reset the map
    void RequestReset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();

    // Reset mvpLocalMapPoints to all map points from mvpLocalKeyFrames
    void UpdateLocalMapPoints();

    // Reset mvpLocalKeyFrames to KeyFrames that share map points with the current frame
    void UpdateLocalMapKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastFrameIdMadeIntoKeyFrame;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    virtual void PrintPrefix(ostream & out) override;

private:

   eTrackingState mState;

   // Change mode flags
   std::mutex mMutexMode;
   bool mbActivateLocalizationMode;
   bool mbDeactivateLocalizationMode;

   // Reset flag
   std::mutex mMutexReset;
   bool mbReset;

   MapperServer * mpMapper;

   unsigned int mId;

   unsigned long mNextKeyFrameId;

   unsigned int mKeyFrameIdSpan;

   unsigned long mNextMapPointId; 

   unsigned int mMapPointIdSpan;

   FrameCalibration * mFC;

   void LoadCameraParameters(cv::FileStorage & settings, eSensor sensor);

   void CheckModeChange();

   void CheckReset();

   void Reset();

   bool NeedNewKeyFrame();

   KeyFrame * CreateNewKeyFrame(Frame & currentFrame, ORB_SLAM2::eSensor sensorType);

   unsigned long NewKeyFrameId();

   unsigned long NewMapPointId();

   void Login();

   void Logout();

    void MapperObserverHandleReset();

    void MapperObserverHandleMapChanged();

    class MapperObserver : public MapObserver
    {
        Tracking * mpTracker;
    public:
        MapperObserver(Tracking * pTracker) : mpTracker(pTracker) {};
        virtual void HandleReset() {mpTracker->MapperObserverHandleReset();};
        virtual void HandleMapChanged(MapChangeEvent & mce) {mpTracker->MapperObserverHandleMapChanged();}
    };

    MapperObserver mMapperObserver;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
