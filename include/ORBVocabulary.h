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


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2_TEAM
{

   class ORBVocabulary : public DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
   {
   public:
      
      ORBVocabulary() : isLoaded(false) {}

      bool GetIsLoaded() const { return isLoaded;}

      bool hasSuffix(const std::string &str, const std::string &suffix) {
         std::size_t index = str.find(suffix, str.size() - suffix.size());
         return (index != std::string::npos);
      }

      bool loadFromFile(const std::string & filename)
      {
         if (hasSuffix(filename, ".txt"))
            isLoaded = loadFromTextFile(filename);
         else
            isLoaded = loadFromBinaryFile(filename);

         return isLoaded;
      }

   private:
      bool isLoaded;
   };

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
