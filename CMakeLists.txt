cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ORB_SLAM2_TEAM CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("CMAKE_BUILD_TYPE == " ${CMAKE_BUILD_TYPE})

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
set_property(DIRECTORY PROPERTY CXX_STANDARD 11)
set_property(DIRECTORY PROPERTY CXX_STANDARD_REQUIRED ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 REQUIRED)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 REQUIRED)
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Intel RealSense2 SDK
# ./cmake_modules/FindRealSense2.cmake
find_package(RealSense2)

# ZeroMQ cpp bindings
# https://github.com/zeromq/cppzmq
find_package(cppzmq 3.4 QUIET)

add_subdirectory("Thirdparty/DBoW2" ${PROJECT_BINARY_DIR}/Thirdparty/DBoW2)
add_subdirectory("Thirdparty/g2o" ${PROJECT_BINARY_DIR}/Thirdparty/g2o)

set(PROJECT_FILES 
   include/Converter.h
   include/Duration.h
   include/Enums.h
   include/Frame.h
   include/FrameCalibration.h
   include/FrameDrawer.h
   include/Initializer.h
   include/KeyFrame.h
   include/KeyFrameDatabase.h
   include/LocalMapping.h
   include/LoopClosing.h
   include/Map.h
   include/MapChangeEvent.h
   include/MapDrawer.h
   include/MapObserver.h
   include/MapPoint.h
   include/MapSubject.h
   include/Mapper.h
   include/MapperObserver.h
   include/MapperServer.h
   include/MapperSubject.h
   include/Messages.h
   include/Optimizer.h
   include/ORBextractor.h
   include/ORBmatcher.h
   include/ORBVocabulary.h
   include/PnPsolver.h
   include/Serializer.h
   include/Sim3Solver.h
   include/Sleep.h
   include/Statistics.h
   include/SyncPrint.h
   include/System.h
   include/Tracking.h
   include/Typedefs.h
   include/Viewer.h
   src/Converter.cc
   src/Frame.cc
   src/FrameCalibration.cc
   src/FrameDrawer.cc
   src/Initializer.cc
   src/KeyFrame.cc
   src/KeyFrameDatabase.cc
   src/LocalMapping.cc
   src/LoopClosing.cc
   src/Map.cc
   src/MapChangeEvent.cc
   src/MapDrawer.cc
   src/Mapper.cc
   src/MapperServer.cc
   src/MapPoint.cc
   src/Optimizer.cc
   src/ORBextractor.cc
   src/ORBmatcher.cc
   src/PnPsolver.cc
   src/Serializer.cc
   src/Sim3Solver.cc
   src/SyncPrint.cc
   src/System.cc
   src/Tracking.cc
   src/Viewer.cc
)

if(cppzmq_FOUND)
   set(PROJECT_FILES 
      ${PROJECT_FILES}
      include/MapperClient.h
      src/MapperClient.cc
   )
endif()

add_library(${PROJECT_NAME} STATIC ${PROJECT_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
   ${Pangolin_INCLUDE_DIR}
   $<BUILD_INTERFACE:${cppzmq_INCLUDE_DIR}>
)
target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2>
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/g2o>
   $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
   ${EIGEN3_INCLUDE_DIR}
)


IF(WIN32)
   set(PROJECT_LIBRARIES
      $<BUILD_INTERFACE:${OpenCV_LIBS}>
      $<BUILD_INTERFACE:${Pangolin_LIBRARIES}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/DBoW2.lib> $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/Thirdparty/DBoW2/$(Configuration)/DBoW2.lib>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/g2o.lib> $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/Thirdparty/g2o/$(Configuration)/g2o.lib>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/ORB_SLAM2_TEAM.lib>
   )
   if(cppzmq_FOUND)
      set(PROJECT_LIBRARIES ${PROJECT_LIBRARIES} $<BUILD_INTERFACE:cppzmq-static>)
   endif()
ELSE(WIN32)
   set(PROJECT_LIBRARIES
      $<BUILD_INTERFACE:${OpenCV_LIBS}>
      $<BUILD_INTERFACE:${Pangolin_LIBRARIES}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/DBoW2.lib> $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/Thirdparty/DBoW2/$(Configuration)/DBoW2.lib>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/g2o.lib> $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/Thirdparty/g2o/$(Configuration)/g2o.lib>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/ORB_SLAM2_TEAM.lib>
   )
   if(cppzmq_FOUND)
      set(PROJECT_LIBRARIES ${PROJECT_LIBRARIES} $<BUILD_INTERFACE:cppzmq-static>)
   endif()
ENDIF(WIN32)

target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBRARIES})

# Build examples

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

add_executable(rgbd_tum_team
Examples/RGB-D/rgbd_tum_team.cc)
target_link_libraries(rgbd_tum_team ${PROJECT_NAME})

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

add_executable(stereo_euroc_team
Examples/Stereo/stereo_euroc_team.cc)
target_link_libraries(stereo_euroc_team ${PROJECT_NAME})

add_executable(stereo_far_team
Examples/Stereo/stereo_far_team.cc)
target_link_libraries(stereo_far_team ${PROJECT_NAME})

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

if(RealSense2_FOUND)
   add_executable(realsense2
   Examples/RealSense2/realsense2.cc)
   target_include_directories(realsense2 PUBLIC ${RealSense2_INCLUDE_DIRS})
   target_link_libraries(realsense2 ${PROJECT_NAME} ${RealSense2_LIBS})

   add_executable(realsense2dual
   Examples/RealSense2/realsense2dual.cc)
   target_include_directories(realsense2dual PUBLIC ${RealSense2_INCLUDE_DIRS})
   target_link_libraries(realsense2dual ${PROJECT_NAME} ${RealSense2_LIBS})
endif()


# experimental client and server
if(cppzmq_FOUND)
   if(RealSense2_FOUND)
      add_executable(realsense2client
      Examples/RealSense2/realsense2client.cc)
      target_include_directories(realsense2client PUBLIC ${RealSense2_INCLUDE_DIRS})
      target_link_libraries(realsense2client ${PROJECT_NAME} ${RealSense2_LIBS})
   endif()

   add_executable(server
   src-server/server.cc)
   target_link_libraries(server ${PROJECT_NAME})
endif()

# build tools

add_executable(tools_bin_vocabulary
tools/bin_vocabulary.cc)
target_link_libraries(tools_bin_vocabulary ${PROJECT_NAME})

add_executable(tools_resize_images
tools/resize_images.cc)
target_link_libraries(tools_resize_images ${OpenCV_LIBS})
set_target_properties(tools_resize_images PROPERTIES CXX_STANDARD 17)

# cmake package config

set(ORB_SLAM2_TEAM_VERSION 1.0.0)
include(GenerateExportHeader)
generate_export_header(ORB_SLAM2_TEAM)
set_property(TARGET ORB_SLAM2_TEAM PROPERTY VERSION ${ORB_SLAM2_TEAM_VERSION})
set_property(TARGET ORB_SLAM2_TEAM PROPERTY SOVERSION 1)
set_property(TARGET ORB_SLAM2_TEAM PROPERTY INTERFACE_ORB_SLAM2_TEAM_MAJOR_VERSION 1)
set_property(TARGET ORB_SLAM2_TEAM APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING ORB_SLAM2_TEAM_MAJOR_VERSION
)
set_property(TARGET ORB_SLAM2_TEAM PROPERTY DEBUG_POSTFIX d)


# install target

if(RealSense2_FOUND)
   if(cppzmq_FOUND)
      set(RUNTIME_TARGETS rgbd_tum rgbd_tum_team stereo_kitti stereo_euroc stereo_euroc_team stereo_far_team mono_tum mono_kitti mono_euroc realsense2 realsense2dual realsense2client server)
   else()
      set(RUNTIME_TARGETS rgbd_tum rgbd_tum_team stereo_kitti stereo_euroc stereo_euroc_team stereo_far_team mono_tum mono_kitti mono_euroc realsense2 realsense2dual)
   endif()
else()
   if(cppzmq_FOUND)
      set(RUNTIME_TARGETS rgbd_tum rgbd_tum_team stereo_kitti stereo_euroc stereo_euroc_team stereo_far_team mono_tum mono_kitti mono_euroc server)
   else()
      set(RUNTIME_TARGETS rgbd_tum rgbd_tum_team stereo_kitti stereo_euroc stereo_euroc_team stereo_far_team mono_tum mono_kitti mono_euroc)
   endif()
endif()

install(TARGETS ${RUNTIME_TARGETS}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(TARGETS ORB_SLAM2_TEAM EXPORT ORB_SLAM2_TEAM_TARGETS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")
install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/Vocabulary/
    DESTINATION vocabulary
    FILES_MATCHING PATTERN "*.bin")


# more cmake package config

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/OrbSlam2TeamConfigVersion.cmake"
  VERSION ${ORB_SLAM2_TEAM_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT ORB_SLAM2_TEAM_TARGETS
  FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/OrbSlam2TeamTargets.cmake"
  NAMESPACE OrbSlam2Team::
)

configure_file(OrbSlam2TeamConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/OrbSlam2TeamConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation cmake)
install(EXPORT ORB_SLAM2_TEAM_TARGETS
  FILE
    OrbSlam2TeamTargets.cmake
  NAMESPACE
    OrbSlam2Team::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/OrbSlam2TeamConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/OrbSlam2TeamConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
