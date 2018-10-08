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


#ifndef SYNCPRINT_H
#define SYNCPRINT_H

#include <mutex>
#include <iostream>
#include <sstream>

namespace ORB_SLAM2
{

using namespace std;

// #define ENABLE_SYNCPRINT

#ifdef ENABLE_SYNCPRINT

class SyncPrint
{
public:

    SyncPrint();

    SyncPrint(const char * prefix);

    SyncPrint(const string & prefix);

    static void Print(const char * prefix, const char * message);

    static void Print(const char * prefix, string & message);

    static void Print(string & prefix, string & message);

    static void Print(const char * prefix, stringstream & message);

    static void Print(stringstream prefix, stringstream & message);

    void Print(const char * message);

    void Print(string & message);

    void Print(stringstream & message);

protected:

    virtual void PrintPrefix(ostream & out);

private:

    static mutex mMutexOutput;

    const string mPrefix;

};

#endif //ENABLE_SYNCPRINT

#ifndef ENABLE_SYNCPRINT

class SyncPrint
{
public:

    SyncPrint() {}

    SyncPrint(const char * prefix) {}

    SyncPrint(const string & prefix) {}

    static void Print(const char * prefix, const char * message) {}

    static void Print(const char * prefix, string & message) {}

    static void Print(string & prefix, string & message) {}

    static void Print(const char * prefix, stringstream & message) {}

    static void Print(stringstream prefix, stringstream & message) {}

    void Print(const char * message) {}

    void Print(string & message) {}

    void Print(stringstream & message) {}

protected:

    virtual void PrintPrefix(ostream & out) {}

private:

    static mutex mMutexOutput;

    const string mPrefix;

};

#endif // ENABLE_SYNCPRINT

} // namespace ORB_SLAM2

#endif // SYNCPRINT_H