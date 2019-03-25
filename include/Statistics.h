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

#include <string>
#include <forward_list>

#ifndef STATISTICS_H
#define STATISTICS_H

namespace ORB_SLAM2_TEAM
{
   using namespace std;

   class Statistics
   {
   public:
      const string & Name;
      const int & N;
      const double & Mean;
      const double & SD;

      Statistics(const char * name, forward_list<double> & samples)
         : Name(mName)
         , N(mN)
         , Mean(mMean)
         , SD(mSD)
         , mName(name)
         , mN(0)
         , mMean(0.0)
         , mSD(0.0)
      {
         for (auto it = samples.begin(), endIt = samples.end(); it != endIt; it++, mN++)
         {
            mMean += *it;
         }
         if (0 != mN)
         {
            mMean /= mN;
            for (auto it = samples.begin(), endIt = samples.end(); it != endIt; it++)
            {
               mSD += (*it - mMean) * (*it - mMean);
            }
            mSD = sqrt(mSD / (mN - 1));
         }
      }

      Statistics(const Statistics & s)
         : Name(mName)
         , N(mN)
         , Mean(mMean)
         , SD(mSD)
         , mName(s.mName)
         , mN(s.mN)
         , mMean(s.mMean)
         , mSD(s.mSD)
      {}

      Statistics & operator =(const Statistics & s)
      {
         mName = s.mName;
         mN = s.mN;
         mMean = s.mMean;
         mSD = s.mSD;
      }

   private:
      string mName;
      int mN;
      double mMean;
      double mSD;
   };

}

#endif // STATISTICS_H