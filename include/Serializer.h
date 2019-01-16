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
#include <vector>
#include <set>

#ifndef SERIALIZER_H
#define SERIALIZER_H

namespace ORB_SLAM2_TEAM
{

   class Serializer
   {
   public:

      // utility function to calculate the serialized size of a cv::Mat
      static size_t GetMatBufferSize(const cv::Mat & mat);

      // utility function to read a 2-D cv::Mat from a pre-allocated memory buffer
      static void * ReadMatrix(void * const buffer, cv::Mat & mat);

      // utility function to write a 2-D cv::Mat to a pre-allocated memory buffer
      static void * WriteMatrix(void * const buffer, const cv::Mat & mat);

      // utility function to calculate the serialized size of a std::vector<cv::KeyPoint>
      static size_t GetKeyPointVectorBufferSize(const std::vector<cv::KeyPoint> & kpv);

      // utility function to read a std::vector<cv::KeyPoint> from a pre-allocated memory buffer
      static void * ReadKeyPointVector(void * const buffer, std::vector<cv::KeyPoint> & kpv);

      // utility function to write a std::vector<cv::KeyPoint> to a pre-allocated memory buffer
      static void * WriteKeyPointVector(void * const buffer, const std::vector<cv::KeyPoint> & kpv);

      // utility template function to read a single value
      template<typename T>
      static void * ReadValue(void * const buffer, T & val);

      // utility template function to write a single value
      template<typename T>
      static void * WriteValue(void * const buffer, const T val);

      // utility template function to calculate the serialized size of a std::vector<T>
      template<typename T>
      static size_t GetVectorBufferSize(const size_t quantity);

      // utility template function to read a std::vector<T> from a pre-allocated memory buffer
      template<typename T>
      static void * ReadVector(void * const buffer, std::vector<T> & v);

      // utility template function to write a std::vector<T> to a pre-allocated memory buffer
      template<typename T>
      static void * WriteVector(void * const buffer, const std::vector<T> & v);

      // utility template function to calculate the serialized size of a std::set<T>
      template<typename T>
      static size_t GetSetBufferSize(const size_t quantity);

      // utility template function to read a std::set<T> from a pre-allocated memory buffer
      template<typename T>
      static void * ReadSet(void * const buffer, std::set<T> & s);

      // utility template function to write a std::set<T> to a pre-allocated memory buffer
      template<typename T>
      static void * WriteSet(void * const buffer, const std::set<T> & s);

   private:

      // total size of the object in bytes when serialized
      virtual size_t GetBufferSize() = 0;

      // reads the data buffer and sets object variables
      virtual void ReadBytes(void * const buffer) = 0;

      // writes object variables to a data buffer
      virtual void WriteBytes(void * const buffer) = 0;

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
   inline void * Serializer::ReadValue(void * const buffer, T & val)
   {
      T * pVal = (T *)buffer;
      val = *pVal;
      return pVal + 1;
   }

   template<typename T>
   inline void * Serializer::WriteValue(void * const buffer, const T val)
   {
      T * pVal = (T *)buffer;
      *pVal = val;
      return pVal + 1;
   }

   template<typename T>
   inline size_t Serializer::GetVectorBufferSize(const size_t quantity)
   {
      return sizeof(size_t) + quantity * sizeof(T);
   }

   template<typename T>
   void * Serializer::ReadVector(void * const buffer, std::vector<T> & v)
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
   void * Serializer::WriteVector(void * const buffer, const std::vector<T> & v)
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

   template<typename T>
   inline size_t Serializer::GetSetBufferSize(const size_t quantity)
   {
      return sizeof(size_t) + quantity * sizeof(T);
   }

   template<typename T>
   void * Serializer::ReadSet(void * const buffer, std::set<T> & s)
   {
      size_t * pQuantity = (size_t *)buffer;
      s.clear();
      T * pData = (T *)(pQuantity + 1);
      for (int i = 0; i < *pQuantity; ++i)
      {
         s.insert(*pData);
         ++pData;
      }
      return pData;
   }

   template<typename T>
   void * Serializer::WriteSet(void * const buffer, const std::set<T> & s)
   {
      size_t * pQuantity = (size_t *)buffer;
      *pQuantity = s.size();
      T * pData = (T *)(pQuantity + 1);
      for (T t : s)
      {
         *pData = t;
         ++pData;
      }
      return pData;
   }
}

#endif // SERIALIZER_H