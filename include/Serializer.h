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

#include <opencv2\core\mat.hpp>

#ifndef SERIALIZER_H
#define SERIALIZER_H

namespace ORB_SLAM2
{

   class Serializer
   {
   public:

      // utility function to calculate the serialized size of a cv::Mat
      static size_t GetMatBufferSize(const cv::Mat & mat);

      // utility function to read a 2-D cv::Mat from a pre-allocated memory buffer
      static void * ReadMatrix(const void * buffer, cv::Mat & mat);

      // utility function to write a 2-D cv::Mat to a pre-allocated memory buffer
      static void * WriteMatrix(const void * buffer, const cv::Mat & mat);

      // utility function to calculate the serialized size of a std::vector<cv::KeyPoint>
      static size_t GetKeyPointVectorBufferSize(const std::vector<cv::KeyPoint> & kpv);

      // utility function to read a std::vector<cv::KeyPoint> from a pre-allocated memory buffer
      static void * ReadKeyPointVector(const void * buffer, std::vector<cv::KeyPoint> & kpv);

      // utility function to write a std::vector<cv::KeyPoint> to a pre-allocated memory buffer
      static void * WriteKeyPointVector(const void * buffer, const std::vector<cv::KeyPoint> & kpv);

      // utility template function to calculate the serialized size of a std::vector<T>
      template<typename T>
      static size_t GetVectorBufferSize(const std::vector<T> & v);

      // utility template function to read a std::vector<T> from a pre-allocated memory buffer
      template<typename T>
      static void * ReadVector(const void * buffer, std::vector<T> & v);

      // utility template function to write a std::vector<T> to a pre-allocated memory buffer
      template<typename T>
      static void * WriteVector(const void * buffer, const std::vector<T> & v);

   private:

      // total size of the object in bytes when serialized
      virtual size_t GetBufferSize() = 0;

      // reads the data buffer and sets object variables
      virtual void ReadBytes(const void * buffer) = 0;

      // writes object variables to a data buffer
      virtual void WriteBytes(const void * buffer) = 0;

      struct MatrixHeader
      {
         int rows;
         int cols;
         int type;
      };

      struct KeyPointItem
      {
         float x;
         float y;
         float size;
         float angle;
         float response;
         int octave;
         int class_id;
      };

   };

   template<typename T>
   inline size_t Serializer::GetVectorBufferSize(const std::vector<T> & v)
   {
      return sizeof(size_t) + v.size() * sizeof(T);
   }

   template<typename T>
   void * Serializer::ReadVector(const void * buffer, std::vector<T> & v)
   {
      size_t * pQuantity = (size_t *)buffer;
      v.resize(*pQuantity);
      T * pData = (T *)(pQuantity + 1);
      for (int i = 0; i < *pQuantity; ++i)
      {
         v.at(i) = *pData;
         ++pData;
      }
      return pData;
   }

   template<typename T>
   void * Serializer::WriteVector(const void * buffer, const std::vector<T> & v)
   {
      size_t * pQuantity = (size_t *)buffer;
      *pQuantity = v.size();
      T * pData = (T *)(pQuantity + 1);
      for (T t : v)
      {
         *pData = t;
         ++pData;
      }
      return pData;
   }
}

#endif // SERIALIZER_H