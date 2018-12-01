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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Mapper.h"
#include "MapChangeEvent.h"
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

   class Tracking : protected SyncPrint
   {

   public:
      Tracking(
         cv::FileStorage & settings,
         ORBVocabulary & vocab,
         Mapper & mapper,
         FrameDrawer* pFrameDrawer,
         MapDrawer* pMapDrawer,
         SensorType sensor
      );

      ~Tracking();

      // Preprocess the input and call Track(). Extract features and performs stereo matching.
      cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);
      cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);
      cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

      void SetViewer(Viewer* pViewer);

      // This stops local mapping thread (map building) and performs only camera tracking.
      void ActivateLocalizationMode();

      // This resumes local mapping thread and performs SLAM again.
      void DeactivateLocalizationMode();

      // Reset the map
      void RequestReset();

   public:

      TrackingState mLastProcessedState;

      // Input sensor
      SensorType mSensor;

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

      // quantity of successful relocalizations (after a tracking failure)
      unsigned int & quantityRelocalizations;

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
      ORBVocabulary & mVocab;

      // Initalization (only for monocular)
      Initializer* mpInitializer;

      // used to hold nearby KeyFrames. different for each Frame.
      std::vector<KeyFrame*> mvpLocalKeyFrames;

      // used to hold MapPoints from nearby KeyFrames. different for each Frame.
      std::vector<MapPoint*> mvpLocalMapPoints;

      //Drawers
      Viewer* mpViewer;
      FrameDrawer* mpFrameDrawer;
      MapDrawer* mpMapDrawer;

      //New KeyFrame rules (according to fps)
      int mMinFrames;
      int mMaxFrames;

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

      TrackingState mState;

      // Change mode flags
      std::mutex mMutexMode;
      bool mbActivateLocalizationMode;
      bool mbDeactivateLocalizationMode;

      // Reset flag
      std::mutex mMutexReset;
      bool mbReset;

      Mapper & mMapper;

      unsigned int mId;

      unsigned long mNextKeyFrameId;

      unsigned int mKeyFrameIdSpan;

      unsigned long mNextMapPointId;

      unsigned int mMapPointIdSpan;

      // includes camera calibration and lens distortion, and other variables shared across frames
      FrameCalibration mFC;

      cv::Mat pivotCal;

      unsigned int mQuantityRelocalizations;

      void LoadCameraParameters(cv::FileStorage & settings, SensorType sensor);

      void CheckModeChange();

      void CheckReset();

      void Reset();

      bool NeedNewKeyFrame();

      KeyFrame * CreateNewKeyFrame(Frame & currentFrame, SensorType sensorType);

      unsigned long NewKeyFrameId();

      unsigned long NewMapPointId();

      void Login();

      void Logout();

      void RotationsYZXtoMat(double y, double z, double x, cv::Mat & m);

      void RotationsYXZtoMat(double y, double x, double z, cv::Mat & m);

      void HandleMapReset();

      class PrivateMapperObserver : public MapperObserver
      {
         Tracking * mpTracker;
      public:
         PrivateMapperObserver(Tracking * pTracker) : mpTracker(pTracker) {};
         virtual void HandleMapReset() { mpTracker->HandleMapReset(); };
      };

      PrivateMapperObserver mMapperObserver;

   };

} //namespace ORB_SLAM

#endif // TRACKING_H
