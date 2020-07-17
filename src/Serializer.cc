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

#include "Serializer.h"
#include <sstream>

namespace ORB_SLAM2_TEAM
{

   using namespace std;

   size_t Serializer::GetMatBufferSize(const cv::Mat & mat)
   {
      //assert(mat.dims == 2);
      if (mat.dims != 2)
      {
         stringstream ss;
         ss <<  "Serializer::GetMatBufferSize only 2-dimensional matrices are supported, dimensions=" << mat.dims;
         throw std::exception(ss.str().c_str());
      }

      return sizeof(MatrixHeader) + mat.rows * mat.cols * mat.elemSize();
   }

   void * Serializer::ReadMatrix(void * const buffer, cv::Mat & mat)
   {
      assert(mat.dims == 2);
      MatrixHeader * pMH = (MatrixHeader *)buffer;
      mat.create(pMH->rows, pMH->cols, pMH->type);
      char * pData = (char *)(pMH + 1);
      if (mat.isContinuous())
      {
         const unsigned int data_size = mat.rows * mat.cols * mat.elemSize();
         memcpy(mat.ptr(), pData, data_size);
         pData += data_size;
      }
      else
      {
         const unsigned int row_size = mat.cols * mat.elemSize();
         for (int i = 0; i < mat.rows; i++)
         {
            memcpy(mat.ptr(i), pData, row_size);
            pData += row_size;
         }
      }
      return pData;
   }

   void * Serializer::WriteMatrix(void * const buffer, const cv::Mat & mat)
   {
      assert(mat.dims == 2);
      MatrixHeader * pMH = (MatrixHeader *)buffer;
      pMH->rows = mat.rows;
      pMH->cols = mat.cols;
      pMH->type = mat.type();
      char * pData = (char *)(pMH + 1);
      if (mat.isContinuous())
      {
         const unsigned int data_size = mat.rows * mat.cols * mat.elemSize();
         memcpy(pData, mat.ptr(), data_size);
         pData += data_size;
      }
      else
      {
         const unsigned int row_size = mat.cols * mat.elemSize();
         for (int i = 0; i < mat.rows; i++) 
         {
            memcpy(pData, mat.ptr(i), row_size);
            pData += row_size;
         }
      }
      return pData;
   }

   size_t Serializer::GetKeyPointVectorBufferSize(const std::vector<cv::KeyPoint> & kpv)
   {
      return sizeof(size_t) + kpv.size() * sizeof(KeyPointItem);
   }

   void * Serializer::ReadKeyPointVector(void * const buffer, std::vector<cv::KeyPoint> & kpv)
   {
      size_t * pQuantity = (size_t *)buffer;
      kpv.resize(*pQuantity);
      KeyPointItem * pData = (KeyPointItem *)(pQuantity + 1);
      for (int i = 0; i < *pQuantity; ++i)
      {
         kpv.at(i) = cv::KeyPoint(
            pData->x, 
            pData->y, 
            pData->size, 
            pData->angle, 
            pData->response, 
            pData->octave, 
            pData->class_id);
         ++pData;
      }
      return pData;
   }

   void * Serializer::WriteKeyPointVector(void * const buffer, const std::vector<cv::KeyPoint> & kpv)
   {
      size_t * pQuantity = (size_t *)buffer;
      *pQuantity = kpv.size();
      KeyPointItem * pData = (KeyPointItem *)(pQuantity + 1);
      for (cv::KeyPoint kp : kpv)
      {
         pData->x = kp.pt.x;
         pData->y = kp.pt.y;
         pData->size = kp.size;
         pData->angle = kp.angle;
         pData->response = kp.response;
         pData->octave = kp.octave;
         pData->class_id = kp.octave;
         ++pData;
      }
      return pData;
   }

}