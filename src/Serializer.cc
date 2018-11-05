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

namespace ORB_SLAM2
{

   size_t Serializer::GetMatBufferSize(const cv::Mat & mat)
   {
      //assert(mat.dims == 2);
      if (mat.dims != 2)
         throw std::exception("only 2-dimensional matrices are supported");

      return sizeof(MatrixHeader) + mat.rows * mat.cols * mat.elemSize();
   }

   void * Serializer::ReadMatrix(const void * buffer, cv::Mat & mat)
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

   void * Serializer::WriteMatrix(const void * buffer, const cv::Mat & mat)
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

}