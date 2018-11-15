/**
* This file is part of ORB-SLAM2-TEAM.
*
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

#ifndef MAPPERCLIENT_H
#define MAPPERCLIENT_H

#include <zmq.hpp>

#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Enums.h"
#include "MapObserver.h"
#include "Mapper.h"
#include "MapperServer.h"

namespace ORB_SLAM2
{

   class LocalMapping;
   class LoopClosing;

   // interface for all Mapping functionality
   class MapperClient : public Mapper, protected SyncPrint
   {
   public:

      MapperClient(cv::FileStorage & settings, ORBVocabulary & vocab, const bool bMonocular);

      virtual long unsigned  KeyFramesInMap();

      virtual void Reset();

      virtual std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame * F);

      virtual bool GetPauseRequested();

      virtual bool AcceptKeyFrames();

      virtual bool InsertKeyFrame(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF);

      virtual void InitializeMono(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2);

      virtual void InitializeStereo(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF);

      virtual bool GetInitialized();

      virtual Map & GetMap();

      virtual std::mutex & GetMutexMapUpdate();

      virtual void LoginTracker(
         const cv::Mat & pivotCalib,
         unsigned int & trackerId,
         unsigned long  & firstKeyFrameId,
         unsigned int & keyFrameIdSpan,
         unsigned long & firstMapPointId,
         unsigned int & mapPointIdSpan);

      virtual void LogoutTracker(unsigned int id);

      virtual void UpdatePose(unsigned int trackerId, const cv::Mat & poseTcw);

      virtual vector<cv::Mat> GetTrackerPoses();

      virtual vector<cv::Mat> GetTrackerPivots();

   private:
      static const unsigned int MAX_TRACKERS = 2;

      static const unsigned int KEYFRAME_ID_SPAN = MAX_TRACKERS;

      /*
      The Local Mapper does not create KeyFrames, but it does create MapPoints. This is why the
      MAPPOINT_ID_SPAN is one more than the KEYFRAME_ID_SPAN. This set of MapPoint Ids is reserved
      for the Local Mapper.
      */
      static const unsigned int MAPPOINT_ID_SPAN = MAX_TRACKERS + 1;

      static const unsigned long FIRST_MAPPOINT_ID_LOCALMAPPER = MAX_TRACKERS;

      struct TrackerStatus {
         bool connected;
         unsigned long nextKeyFrameId;
         unsigned long nextMapPointId;
      };

      TrackerStatus mTrackers[MAX_TRACKERS];

      std::mutex mMutexLogin;

      std::mutex mMutexMapUpdate;

      std::mutex mMutexSocketSub;

      ORBVocabulary & mVocab;

      bool mbMonocular;

      Map mMap;

      bool mInitialized;

      //MapperServer mServer;

      string mServerAddress;

      string mPublisherAddress;

      zmq::context_t mContext;

      zmq::socket_t mSocketReq;

      zmq::socket_t mSocketSub;

      zmq::message_t RequestReply(zmq::message_t & request);

      void GreetServer();

      void LoginTrackerServer(
         const cv::Mat & pivotCalib,
         unsigned int & trackerId,
         id_type  & firstKeyFrameId,
         unsigned int & keyFrameIdSpan,
         id_type & firstMapPointId,
         unsigned int & mapPointIdSpan);

      void GetMapFromServer(const unsigned int trackerId);

      void InitializeMonoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF1, KeyFrame * pKF2);

      void InitializeStereoServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF);

      bool InsertKeyFrameServer(unsigned int trackerId, vector<MapPoint *> & mapPoints, KeyFrame * pKF);

      void MapperServerObserverReset();

      void MapperServerObserverMapChanged(MapChangeEvent & mce);

      class MapperServerObserver : public MapObserver
      {
         MapperClient * mpMapperClient;
      public:
         MapperServerObserver(MapperClient * pMapperClient) : mpMapperClient(pMapperClient) {};
         virtual void HandleReset() { mpMapperClient->MapperServerObserverReset(); };
         virtual void HandleMapChanged(MapChangeEvent & mce) { mpMapperClient->MapperServerObserverMapChanged(mce); }
      };

      MapperServerObserver mMapperServerObserver;
   };

}

#endif // MAPPERCLIENT_H