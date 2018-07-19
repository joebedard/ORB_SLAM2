cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ORB_SLAM2 CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("CMAKE_BUILD_TYPE ==" ${CMAKE_BUILD_TYPE})

IF(CMAKE_COMPILER_IS_GNUCXX)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

  IF(MSVC)
    MESSAGE(STATUS "Compiling with MSVC")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS=1)  # Avoid deprecated fprintf(), etc.
    add_definitions("/nologo")
    add_definitions("/wd4267") # conversion from 'size_t' to 'GLsizei'
    add_definitions("/wd4305") # truncation from 'double' to 'float'
    add_definitions("/wd4244") # conversion from 'double' to 'float'
  ENDIF(MSVC)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set_property(DIRECTORY PROPERTY CXX_STANDARD 11)
set_property(DIRECTORY PROPERTY CXX_STANDARD_REQUIRED ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(RealSense2 REQUIRED)
find_package(ZeroMQ REQUIRED)

add_subdirectory("Thirdparty/DBoW2" ${PROJECT_BINARY_DIR}/Thirdparty/DBoW2)
add_subdirectory("Thirdparty/g2o" ${PROJECT_BINARY_DIR}/Thirdparty/g2o)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${RealSense2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
include/Converter.h
include/Enums.h
include/Frame.h
include/FrameDrawer.h
include/Initializer.h
include/KeyFrame.h
include/KeyFrameDatabase.h
include/LocalMapping.h
include/LoopClosing.h
include/Map.h
include/MapDrawer.h
include/Mapper.h
include/MapPoint.h
include/Optimizer.h
include/ORBextractor.h
include/ORBmatcher.h
include/ORBVocabulary.h
include/PnPsolver.h
include/Sim3Solver.h
include/Sleep.h
include/System.h
include/Tracking.h
include/Viewer.h
src/Converter.cc
src/Frame.cc
src/FrameDrawer.cc
src/Initializer.cc
src/KeyFrame.cc
src/KeyFrameDatabase.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/Map.cc
src/MapDrawer.cc
src/Mapper.cc
src/MapPoint.cc
src/Optimizer.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/PnPsolver.cc
src/Sim3Solver.cc
src/System.cc
src/Tracking.cc
src/Viewer.cc
)

IF(WIN32)
   SET(ORB_SLAM_LINK_LIBS 
      ${PROJECT_BINARY_DIR}/Thirdparty/DBoW2/$(Configuration)/DBoW2.lib
      ${PROJECT_BINARY_DIR}/Thirdparty/g2o/$(Configuration)/g2o.lib)
ELSE(WIN32)
   SET(ORB_SLAM_LINK_LIBS 
      ${PROJECT_BINARY_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
      ${PROJECT_BINARY_DIR}/Thirdparty/g2o/lib/libg2o.so)
ENDIF(WIN32)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM_LINK_LIBS}
${ZeroMQ_LIBS}
)

# Build examples

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(realsense2
Examples/RealSense2/realsense2.cc)
target_link_libraries(realsense2 ${PROJECT_NAME} ${RealSense2_LIBS})

# build client and server

add_executable(client
src-client/client.cc)
target_link_libraries(client ${PROJECT_NAME} ${RealSense2_LIBS})

add_executable(server
src-server/server.cc)
target_link_libraries(server ${PROJECT_NAME} ${RealSense2_LIBS})
