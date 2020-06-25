# - Try to find Intel RealSense SDK 2.0
#
# This module does not support required versions in find_package. Valid examples:
#   find_package(RealSense2)
#   find_package(RealSense2 QUIET)
#   find_package(RealSense2 REQUIRED)
# This module only looks for version 2.0 of the SDK.
#
# Once done this will define
#
#  RealSense2_FOUND - system has Intel RealSense SDK 2.0
#  RealSense2_DIR - base directory of the SDK
#  RealSense2_INCLUDE_DIRS - one or more include directories
#  RealSense2_LIBS - one or more libraries

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

if (WIN32)
   find_path(RealSense2_DIR 
      NAMES intel.realsense.props
      PATHS
      "C:/Program Files (x86)"
      "C:/Program Files"
      PATH_SUFFIXES 
      "Intel RealSense SDK 2.0"
      DOC "base directory of Intel RealSense SDK 2.0"
   )

   find_package_handle_standard_args(RealSense2 DEFAULT_MSG RealSense2_DIR)

   find_path(RealSense2_INCLUDE_DIRS 
      NAMES librealsense2
      PATHS
      ${RealSense2_DIR}
      PATH_SUFFIXES 
      "include"
   )

   find_file(RealSense2_LIBS 
      NAMES realsense2.lib
      PATHS
      ${RealSense2_DIR}
      PATH_SUFFIXES 
      "lib/x64"
   )
endif (WIN32)

mark_as_advanced(RealSense2_INCLUDE_DIRS)
mark_as_advanced(RealSense2_LIBS)

find_package_handle_standard_args(RealSense2 DEFAULT_MSG RealSense2_INCLUDE_DIRS RealSense2_LIBS)

#set(RealSense2_FOUND 1)
