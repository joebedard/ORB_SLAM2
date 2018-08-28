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


#include "SyncPrint.h"

#ifdef ENABLE_SYNCPRINT

namespace ORB_SLAM2
{

using namespace std;

mutex SyncPrint::mMutexOutput;

SyncPrint::SyncPrint() {};

SyncPrint::SyncPrint(const char * prefix) : mPrefix(prefix) {};

SyncPrint::SyncPrint(const string & prefix) : mPrefix(prefix) {};

//static
void SyncPrint::Print(const char * prefix, const char * message)
{
    unique_lock<mutex> lock(mMutexOutput);

    if (prefix)
        cout << prefix;

    if (message)
        cout << message;

    cout << endl;
}

//static
void SyncPrint::Print(const char * prefix, string & message)
{
    Print(prefix, message.c_str());
}

//static
void SyncPrint::Print(string & prefix, string & message)
{
    Print(prefix.c_str(), message.c_str());
}

//static
void SyncPrint::Print(const char * prefix, stringstream & message)
{
    Print(prefix, message.str().c_str());
}

//static
void SyncPrint::Print(stringstream prefix, stringstream & message)
{
    Print(prefix.str().c_str(), message.str().c_str());
}

void SyncPrint::Print(const char * message)
{
    unique_lock<mutex> lock(mMutexOutput);

    PrintPrefix(cout);

    if (message)
        cout << message;

    cout << endl;
}

void SyncPrint::Print(string & message)
{
    Print(message.c_str());
}

void SyncPrint::Print(stringstream & message)
{
    Print(message.str().c_str());
}

void SyncPrint::PrintPrefix(ostream & out)
{
    out << mPrefix;
}

}

#endif // ENABLE_SYNCPRINT
