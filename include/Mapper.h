/**
* This file is part of ORB-SLAM2.
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

#ifndef MAPPER_H
#define MAPPER_H

#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Enums.h"

namespace ORB_SLAM2
{

   class LocalMapping;
   class LoopClosing;

   // interface for all Mapping functionality
   class Mapper
   {
   public:

      Mapper(Map * pMap, ORBVocabulary* pVocab, const bool bMonocular);

      virtual std::mutex & getMutexMapUpdate();

      virtual long unsigned  KeyFramesInMap();

      virtual void Reset();

      virtual std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

      virtual bool stopRequested();

      virtual bool AcceptKeyFrames();

      virtual void InsertKeyFrame(KeyFrame* pKF);

      virtual void Shutdown();

      virtual KeyFrame * CreateNewKeyFrame(Frame & currentFrame, ORB_SLAM2::eSensor sensorType);

      virtual void Initialize(Map & pMap);

      virtual bool GetInitialized();

   private:
      ORBVocabulary * mpVocab;
      bool mbMonocular;
      KeyFrameDatabase * mpKeyFrameDB;
      Map* mpMap;
      static long unsigned int nNextMapPointId;

      // initialization variables
      bool mInitialized;
      LocalMapping* mpLocalMapper;
      LoopClosing* mpLoopCloser;

      // System threads: Local Mapping, Loop Closing
      std::thread* mptLocalMapping;
      std::thread* mptLoopClosing;
   };

}

#endif // MAPPER_H