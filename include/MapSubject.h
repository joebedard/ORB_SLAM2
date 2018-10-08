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

#ifndef MAPSUBJECT_H
#define MAPSUBJECT_H

#include <map>
#include "MapObserver.h"

namespace ORB_SLAM2
{

    /*
        base class for Subject class from the Observer Pattern
        supports a shared collection of observers with thread-safe methods 
        see https://sourcemaking.com/design_patterns/observer
    */
    class MapSubject
    {
    public:

        MapSubject()
        {
            mObservers = new std::map<MapObserver *, MapObserver *>();
        }

        // allows a shared collection of observers
        MapSubject(MapSubject & copy)
        {
            mObservers = copy.mObservers;
        }

        void AddObserver(MapObserver * ob)
        {
            unique_lock<mutex> lock(mMutex);

            (*mObservers)[ob] = ob;
        }

        void RemoveObserver(MapObserver * ob)
        {
            unique_lock<mutex> lock(mMutex);

            (*mObservers).erase(ob);
        }

    protected:

        void NotifyReset()
        {
            unique_lock<mutex> lock(mMutex);

            for (auto it : (*mObservers))
            {
                it.second->HandleReset();
            }
        }

        void NotifyMapChanged(MapChangeEvent & mce)
        {
            unique_lock<mutex> lock(mMutex);

            for (auto it : (*mObservers))
            {
                it.second->HandleMapChanged(mce);
            }
        }

    private:

        std::mutex mMutex;

        std::map<MapObserver *, MapObserver *> * mObservers;

    };

}

#endif // MAPSUBJECT_H