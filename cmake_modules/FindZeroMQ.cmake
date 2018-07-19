# - Try to find Intel RealSense SDK 2.0
#
# This module does not support required versions in find_package. Valid examples:
#   find_package(ZeroMQ)
#   find_package(ZeroMQ QUIET)
#   find_package(ZeroMQ REQUIRED)
# This module only looks for version 2.0 of the SDK.
#
# Once done this will define
#
#  ZeroMQ_FOUND - system has Intel RealSense SDK 2.0
#  ZeroMQ_DIR - base directory of the SDK
#  ZeroMQ_INCLUDE_DIRS - one or more include directories
#  ZeroMQ_LIBS - one or more libraries

# This file is part of ORB-SLAM2-CS.
#
# Copyright (C) 2018 Joe Bedard <mr dot joe dot bedard at gmail dot com>
# For more information see <https://github.com/joebedard/ORB_SLAM2_CS>
#
# ORB-SLAM2-CS is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ORB-SLAM2-CS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ORB-SLAM2-CS. If not, see <http://www.gnu.org/licenses/>.

include(FindPackageHandleStandardArgs)

find_path(ZeroMQ_DIR 
   NAMES COPYING.LESSER COPYING
   PATHS
   "/libzmq"
   DOC "base directory of ZeroMQ source distribution, usually called libzmq"
)

find_package_handle_standard_args(ZeroMQ DEFAULT_MSG ZeroMQ_DIR)

find_path(ZeroMQ_INCLUDE_DIRS 
   NAMES zmq.h
   PATHS
   ${ZeroMQ_DIR}
   PATH_SUFFIXES 
   "include"
)

if (WIN32)
   find_file(ZeroMQ_LIBS 
      NAMES libzmq.lib
      PATHS
      ${ZeroMQ_DIR}
      PATH_SUFFIXES 
      "bin/Win32/Release/v141/static"
   )
endif (WIN32)

mark_as_advanced(ZeroMQ_INCLUDE_DIRS)
mark_as_advanced(ZeroMQ_LIBS)

find_package_handle_standard_args(ZeroMQ DEFAULT_MSG ZeroMQ_INCLUDE_DIRS ZeroMQ_LIBS)

set(ZeroMQ_FOUND 1)
