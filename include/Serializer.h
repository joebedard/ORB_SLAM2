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
   };
}

#endif // SERIALIZER_H