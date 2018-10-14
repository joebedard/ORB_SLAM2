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


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "Converter.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "Sleep.h"

#include <iostream>
#include <mutex>
#include <cmath>

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(
    ORBVocabulary* pVoc,
    FrameDrawer* pFrameDrawer,
    MapDrawer* pMapDrawer,
    MapperServer* pMapper,
    cv::FileStorage & fSettings,
    eSensor sensor
) :
    SyncPrint("Tracking: "),
    mpORBVocabulary(pVoc),
    mpFrameDrawer(pFrameDrawer), 
    mpMapDrawer(pMapDrawer),
    mpMapper(pMapper),
    mSensor(sensor),
    mId(-1),
    mbOnlyTracking(false), 
    mbReset(false),
    mpInitializer(static_cast<Initializer*>(NULL)),
    mpViewer(NULL),
    mnLastRelocFrameId(0),
    mbActivateLocalizationMode(false), 
    mbDeactivateLocalizationMode(false), 
    mState(NOT_INITIALIZED),
    mNextKeyFrameId(0),
    mKeyFrameIdSpan(0), 
    mNextMapPointId(0),
    mMapPointIdSpan(0),
    mMapperObserver(this),
    pivotCal(4, 4, CV_32F)
{
    LoadCameraParameters(fSettings, sensor);
    Login();
    mpMapper->AddObserver(&mMapperObserver);
}

Tracking::~Tracking()
{
    mpMapper->LogoutTracker(mId);
    delete mFC;
}

void Tracking::PrintPrefix(ostream & out)
{
    SyncPrint::PrintPrefix(out);
    out << "Id=" << mId << " ";
}

void Tracking::LoadCameraParameters(cv::FileStorage & fSettings, eSensor sensor)
{ 
    // Load camera parameters from settings file

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    int width = (int)fSettings["Camera.width"];
    if (0 == width)
        throw new exception("Camera.width is not set.");

    int height = (int)fSettings["Camera.height"];
    if (0 == height)
        throw new exception("Camera.height is not set.");

    mFC = new FrameCalibration(K, DistCoef, FrameCalibration::ImageBounds(K, DistCoef, width, height));

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    stringstream ss;
    ss << "\n" << "Camera Parameters: " << "\n";
    ss << "- fx: " << fx << "\n";
    ss << "- fy: " << fy << "\n";
    ss << "- cx: " << cx << "\n";
    ss << "- cy: " << cy << "\n";
    ss << "- k1: " << DistCoef.at<float>(0) << "\n";
    ss << "- k2: " << DistCoef.at<float>(1) << "\n";
    if(DistCoef.rows==5)
        ss << "- k3: " << DistCoef.at<float>(4) << "\n";
    ss << "- p1: " << DistCoef.at<float>(2) << "\n";
    ss << "- p2: " << DistCoef.at<float>(3) << "\n";
    ss << "- fps: " << fps << "\n";


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        ss << "- color order: RGB (ignored if grayscale)" << "\n";
    else
        ss << "- color order: BGR (ignored if grayscale)" << "\n";

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    ss << "\n"  << "ORB Extractor Parameters: " << "\n";
    ss << "- Number of Features: " << nFeatures << "\n";
    ss << "- Scale Levels: " << nLevels << "\n";
    ss << "- Scale Factor: " << fScaleFactor << "\n";
    ss << "- Initial Fast Threshold: " << fIniThFAST << "\n";
    ss << "- Minimum Fast Threshold: " << fMinThFAST << "\n";

    if(sensor==STEREO || sensor==RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        ss << "\n" << "Depth Threshold (Close/Far Points): " << mThDepth << "\n";
    }

    if(sensor==RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    // Load Pivot Calibration

    float pivotRX = (float)fSettings["Pivot.rx"];
    float pivotRY = (float)fSettings["Pivot.ry"];
    float pivotRZ = (float)fSettings["Pivot.rz"];
    float pivotTX = (float)fSettings["Pivot.tx"];
    float pivotTY = (float)fSettings["Pivot.ty"];
    float pivotTZ = (float)fSettings["Pivot.tz"];

    ss << "\n" << "Pivot Calibration: " << "\n";
    ss << "- X rotation: " << pivotRX << "\n";
    ss << "- Y rotation: " << pivotRY << "\n";
    ss << "- Z rotation: " << pivotRZ << "\n";
    ss << "- X translation: " << pivotTX << "\n";
    ss << "- Y translation: " << pivotTY << "\n";
    ss << "- Z translation: " << pivotTZ << "\n";

    pivotCal = cv::Mat::eye(4, 4, CV_32F);
    RotationsYXZtoMat(pivotRY, pivotRX, pivotRZ, pivotCal);
    pivotCal.at<float>(0, 3) = pivotTX;
    pivotCal.at<float>(1, 3) = pivotTY;
    pivotCal.at<float>(2, 3) = pivotTZ;
    pivotCal.at<float>(3, 3) = 1.0f;
    pivotCal.at<float>(3, 2) = 0.0f;
    pivotCal.at<float>(3, 1) = 0.0f;
    pivotCal.at<float>(3, 0) = 0.0f;

    ss << "Mat: " << pivotCal << "\n";
    Print(ss);
}

/*
   Order of euler angles: yaw first, then pitch, then roll
   matrix row column ordering:
   [m00 m01 m02]
   [m10 m11 m12]
   [m20 m21 m22]
*/
void Tracking::RotationsYXZtoMat(double y, double x, double z, cv::Mat & m)
{
    // Assuming the angles are in radians.
    double cx = std::cos(x);
    double sx = std::sin(x);
    double cy = std::cos(y);
    double sy = std::sin(y);
    double cz = std::cos(z);
    double sz = std::sin(z);

    m.at<float>(0, 0) = cy * cz + sx * sy * sz;
    m.at<float>(0, 1) = sx * sy * cz - cy * sz;
    m.at<float>(0, 2) = cx * sy;
    m.at<float>(1, 0) = cx * sz;
    m.at<float>(1, 1) = cx * cz;
    m.at<float>(1, 2) = -sx;
    m.at<float>(2, 0) = sx * cy * sz - sy * cz;
    m.at<float>(2, 1) = sy * sz + sx * cy * cz;
    m.at<float>(2, 2) = cx * cy;
}

/*
   Order of euler angles: yaw first, then roll, then pitch
   matrix row column ordering:
   [m00 m01 m02]
   [m10 m11 m12]
   [m20 m21 m22]
*/
void Tracking::RotationsYZXtoMat(double y, double z, double x, cv::Mat & m)
{
    // Assuming the angles are in radians.
    double cx = std::cos(x);
    double sx = std::sin(x);
    double cy = std::cos(y);
    double sy = std::sin(y);
    double cz = std::cos(z);
    double sz = std::sin(z);

    m.at<float>(0, 0) = cy * cz;
    m.at<float>(0, 1) = sy * sx - cy * sz * cx;
    m.at<float>(0, 2) = cy * sz * sx + sy * cx;
    m.at<float>(1, 0) = sz;
    m.at<float>(1, 1) = cz * cx;
    m.at<float>(1, 2) = -cz * sx;
    m.at<float>(2, 0) = -sy * cz;
    m.at<float>(2, 1) = sy * sz * cx + cy * sx;
    m.at<float>(2, 2) = -sy * sz * sx + cy * cx;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    CheckModeChange();
    CheckReset();

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mFC, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
   CheckModeChange();
   CheckReset();

   mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mFC, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
   CheckModeChange();
   CheckReset();

   mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if (!mpMapper->GetInitialized())
        mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mFC, mbf, mThDepth);
    else
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mFC, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMapper->GetMap()->mMutexMapUpdate);

    Print("begin Track");
    mLastProcessedState = mState;

    Print("if (!mpMapper->GetInitialized())");
    if (!mpMapper->GetInitialized())
    {
        if (0 == mId)
        {
            if(mSensor==STEREO || mSensor==RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this, mpMapper->GetMap());

            if (!mpMapper->GetInitialized())
            {
                Print("end Track 1");
                return;
            }
        }
        else
        {
            Print("end Track 2");
            return;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        Print("if (mState == TRACKING_OK)");
        if (mState == TRACKING_OK)
        {
            // Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
            {
                bOK = TrackReferenceKeyFrame();
            }
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackReferenceKeyFrame();
            }
        }
        else
        {
            bOK = Relocalization();
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
            bOK = TrackLocalMap();

        if(bOK)
           mState = TRACKING_OK;
        else
           mState = TRACKING_LOST;

        // Update drawer
        mpFrameDrawer->Update(this, mpMapper->GetMap());

        // If tracking was good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
            {
                mVelocity = cv::Mat();
            }

            mpMapper->UpdatePose(mId, mCurrentFrame.mTcw);
            if (mpMapDrawer)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
            {
                KeyFrame * pKF = CreateNewKeyFrame(mCurrentFrame, mSensor);
                if (pKF)
                {
                   mpReferenceKF = pKF;
                   mCurrentFrame.mpReferenceKF = pKF;
                   mnLastFrameIdMadeIntoKeyFrame = mCurrentFrame.mnId;
                   mpLastKeyFrame = pKF;
                }
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // and this tracker is the initializer
        if (mState == TRACKING_LOST && mId == 0)
        {
            if(mpMapper->KeyFramesInMap()<=5)
            {
                Print("Tracking lost soon after initialisation, reseting...");
                RequestReset();
                Print("end Track 3");
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    Print("if (mState == TRACKING_OK)");
    if (mState == TRACKING_OK)
    {
        Print("cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();");
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //Print("mlRelativeFramePoses.push_back(Tcr);");
        mlRelativeFramePoses.push_back(Tcr);
        //Print("mlpReferences.push_back(mpReferenceKF);");
        mlpReferences.push_back(mpReferenceKF);
        //Print("mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);");
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        //Print("mlbLost.push_back(mState == TRACKING_LOST);");
        mlbLost.push_back(mState == TRACKING_LOST);
    }
    else
    {
        // This can happen if tracking is lost
        Print("if (!mlRelativeFramePoses.empty())");
        if (!mlRelativeFramePoses.empty())
        {
            Print("mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());");
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            //Print("mlpReferences.push_back(mlpReferences.back());");
            mlpReferences.push_back(mlpReferences.back());
            //Print("mlFrameTimes.push_back(mlFrameTimes.back());");
            mlFrameTimes.push_back(mlFrameTimes.back());
            //Print("mlbLost.push_back(mState == TRACKING_LOST);");
            mlbLost.push_back(mState == TRACKING_LOST);
        }
    }
    Print("end Track 4");
}


void Tracking::StereoInitialization()
{
    Print("begin StereoInitialization");
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(NewKeyFrameId(), mCurrentFrame);
        vector<KeyFrame *> keyframes;
        keyframes.push_back(pKFini);

        // Create MapPoints and associate to KeyFrame
        vector<MapPoint *> points;
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(NewMapPointId(), x3D, pKFini);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                points.push_back(pNewMP);
                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        stringstream ss;
        ss << "New map created with " << points.size() << " points";
        Print(ss.str().c_str());

        mCurrentFrame.mpReferenceKF = pKFini;
        mLastFrame = Frame(mCurrentFrame);
        mnLastFrameIdMadeIntoKeyFrame = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = points;
        mpReferenceKF = pKFini;

        mpMapper->Initialize(mId, points, keyframes);
        mState = TRACKING_OK;

        if (mpMapDrawer)
        {
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            mpMapDrawer->SetReferenceMapPoints(mvpLocalMapPoints);
        }
    }
    Print("end StereoInitialization");
}

void Tracking::MonocularInitialization()
{
    Print("begin MonocularInitialization");
    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            Print("end MonocularInitialization 1");
            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            Print("end MonocularInitialization 2");
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            Print("end MonocularInitialization 3");
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
    Print("end MonocularInitialization 4");
}

void Tracking::CreateInitialMapMonocular()
{
    Map * pMap = new Map();
    
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(NewKeyFrameId(), mInitialFrame);
    KeyFrame* pKFcur = new KeyFrame(NewKeyFrameId(), mCurrentFrame);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    pMap->AddKeyFrame(pKFini);
    pMap->AddKeyFrame(pKFcur);
    vector<KeyFrame *> keyframes;
    keyframes.push_back(pKFini);
    keyframes.push_back(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    vector<MapPoint *> points;
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(NewMapPointId(), worldPos, pKFcur);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        pMap->AddMapPoint(pMP);
        points.push_back(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    stringstream ss;
    ss << "New Map created with " << points.size() << " points";
    Print(ss.str().c_str());

    Optimizer::GlobalBundleAdjustment(pMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        stringstream ss;
        ss << "Wrong initialization, reseting...";
        Print(ss.str().c_str());

        if (mpInitializer)
        {
           delete mpInitializer;
           mpInitializer = static_cast<Initializer*>(NULL);
        }
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mvpLocalMapPoints = points;
    mpMapper->Initialize(mId, points, keyframes);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastFrameIdMadeIntoKeyFrame = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    if (mpMapDrawer)
    {
        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());
        mpMapDrawer->SetReferenceMapPoints(mvpLocalMapPoints);
    }

    delete pMap;
    mState = TRACKING_OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    Print("begin CheckReplacedInLastFrame");
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
    Print("end CheckReplacedInLastFrame");
}


bool Tracking::TrackReferenceKeyFrame()
{
    Print("begin TrackReferenceKeyFrame");
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        Print("end TrackReferenceKeyFrame 1");
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    Print("end TrackReferenceKeyFrame 2");
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackWithMotionModel()
{
    Print("begin TrackWithMotionModel");
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==MONOCULAR);
    }

    if(nmatches<20)
    {
        Print("end TrackWithMotionModel 1");
        return false;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    
    Print("end TrackWithMotionModel 2");
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    Print("begin TrackLocalMap");
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    // post: potential map points are in mvpLocalMapPoints
    UpdateLocalMap();

    // post: matching map points are in mCurrentFrame.mvpMapPoints
    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    mnMatchesInliers++;
            }
            else if(mSensor==STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    {
        Print("end TrackLocalMap 1");
        return false;
    }

    if(mnMatchesInliers<30)
    {
        Print("end TrackLocalMap 2");
        return false;
    }
    else
    {
        Print("end TrackLocalMap 3");
        return true;
    }
}


bool Tracking::NeedNewKeyFrame()
{
    Print("begin NeedNewKeyFrame");
    if(mbOnlyTracking)
    {
        Print("end NeedNewKeyFrame 1");
        return false;
    }

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpMapper->GetPauseRequested())
    {
        Print("end NeedNewKeyFrame 2");
        return false;
    }

    const int nKFs = mpMapper->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        Print("end NeedNewKeyFrame 3");
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if (mSensor == MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= (mnLastFrameIdMadeIntoKeyFrame + mMaxFrames);

    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= (mnLastFrameIdMadeIntoKeyFrame + mMinFrames)) && mpMapper->AcceptKeyFrames();

    // Condition 1c: tracking is weak
    const bool c1c = (mSensor != MONOCULAR) && (mnMatchesInliers < (nRefMatches * 0.25) || bNeedToInsertClose);

    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < (nRefMatches * thRefRatio)) || bNeedToInsertClose) && (mnMatchesInliers > 15);

    Print("end NeedNewKeyFrame 4");
    return ((c1a || c1b || c1c) && c2);
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    if (mpMapDrawer)
        mpMapDrawer->SetReferenceMapPoints(mvpLocalMapPoints);

    // post: relevant keyframes are in mvpLocalKeyFrames
    UpdateLocalMapKeyFrames();

    // post: potential map points are in mvpLocalMapPoints
    UpdateLocalMapPoints();
}

void Tracking::UpdateLocalMapPoints()
{
   mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalMapKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
    {
        return;
    }

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    Print("begin Relocalization");
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpMapper->DetectRelocalizationCandidates(&mCurrentFrame);
    
    if(vpCandidateKFs.empty())
    {
        Print("end Relocalization 1");
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        Print("end Relocalization 2");
        return false;
    }
    else
    {
        Print("end Relocalization 3");
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::RequestReset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void Tracking::Reset()
{
   Print("System Reseting");
   
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped() && !mpViewer->isFinished())
            sleep(3000);
    }

    {
        unique_lock<mutex> lock(mpMapper->GetMap()->mMutexMapUpdate);
        mpMapper->Reset();
    }


    Frame::nNextId = 0;

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ActivateLocalizationMode()
{
   unique_lock<mutex> lock(mMutexMode);
   mbActivateLocalizationMode = true;
}

void Tracking::DeactivateLocalizationMode()
{
   unique_lock<mutex> lock(mMutexMode);
   mbDeactivateLocalizationMode = true;
}

void Tracking::CheckModeChange()
{
   unique_lock<mutex> lock(mMutexMode);
   if (mbActivateLocalizationMode)
   {
      // keyframes will not be created
      mbOnlyTracking = true;
      mbActivateLocalizationMode = false;
   }
   if (mbDeactivateLocalizationMode)
   {
      // keyframes will be created
      mbOnlyTracking = false;
      mbDeactivateLocalizationMode = false;
   }
}

void Tracking::CheckReset()
{
    Print("begin Reset");
   unique_lock<mutex> lock(mMutexReset);
   if (mbReset)
   {
      Reset();
      mbReset = false;
   }
   Print("end Reset");
}

KeyFrame * Tracking::CreateNewKeyFrame(Frame & currentFrame, ORB_SLAM2::eSensor sensorType)
{
    Print("begin CreateNewKeyFrame");
    KeyFrame* pKF = new KeyFrame(NewKeyFrameId(), currentFrame);

    // Some KeyPoints (features) don't yet have MapPoints, se we create them.
    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    vector<pair<float, int> > vDepthIdx;
    vector<MapPoint *> points;

    if (sensorType != MONOCULAR)
    {
        currentFrame.UpdatePoseMatrices();

        vDepthIdx.reserve(currentFrame.N);
        for (int i = 0; i < currentFrame.N; i++)
        {
            float z = currentFrame.mvDepth[i];
            if (z>0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            for (size_t j = 0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;
                MapPoint* pMP = currentFrame.mvpMapPoints[i];
                if (!pMP || pMP->Observations() < 1)
                {
                    cv::Mat x3D = currentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(NewMapPointId(), x3D, pKF);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    points.push_back(pNewMP);
                }

                if (vDepthIdx[j].first > currentFrame.mThDepth && j > 99)
                    break;
            }
        }
    }

    if (mpMapper->InsertKeyFrame(mId, points, pKF))
    {
        if (sensorType != MONOCULAR)
        {
            // Add new MapPoints to currentFrame. They will be used later by Tracking::TrackWithMotionModel.
            for (size_t j = 0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;
                MapPoint * pMP = pKF->GetMapPoint(i);
                if (!pMP || pMP->Observations() < 1)
                    currentFrame.mvpMapPoints[i] = pMP;
                if (vDepthIdx[j].first > currentFrame.mThDepth && j > 99)
                    break;
            }
        }
        Print("end CreateNewKeyFrame 1");
        return pKF;
    }
    else
    {
        // delete KeyFrame and MapPoints
        delete pKF;
        Print("end CreateNewKeyFrame 2");
        return NULL;
    }
}

unsigned long Tracking::NewKeyFrameId()
{
    unsigned long temp = mNextKeyFrameId;
    mNextKeyFrameId += mKeyFrameIdSpan;
    return temp;
}

unsigned long Tracking::NewMapPointId()
{
    unsigned long temp = mNextMapPointId;
    mNextMapPointId += mMapPointIdSpan;
    return temp;
}

void Tracking::Login()
{
    mId = mpMapper->LoginTracker(mNextKeyFrameId, mKeyFrameIdSpan, mNextMapPointId, mMapPointIdSpan, pivotCal);
    assert(mKeyFrameIdSpan != 0);
    assert(mMapPointIdSpan != 0);

    stringstream ss;
    ss << "Tracking: Login Complete \n";
    ss << "   Tracker Id =         " << mId << "\n";
    ss << "   First Keyframe Id =  " << mNextKeyFrameId << "\n";
    ss << "   Keyframe Id Span =   " << mKeyFrameIdSpan << "\n";
    ss << "   First Map Point Id = " << mNextMapPointId << "\n";
    ss << "   Map Point Id Span =  " << mMapPointIdSpan << "\n";
    Print(ss);
}

void Tracking::Logout()
{
    mpMapper->LogoutTracker(mId);
    mId = -1; //set to max positive integer

    stringstream ss;
    ss << "Tracking: Logout Complete \n";
    ss << "   Tracker Id = " << mId << "\n";
    Print(ss);
}

void Tracking::MapperObserverHandleReset()
{
    if (mpFrameDrawer)
        mpFrameDrawer->Reset();
    if (mpMapDrawer)
        mpMapDrawer->Reset();

    Logout();
    Login();

    if (mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mState = NOT_INITIALIZED;
}

void Tracking::MapperObserverHandleMapChanged()
{
    // TODO - complete this
}

} //namespace ORB_SLAM
