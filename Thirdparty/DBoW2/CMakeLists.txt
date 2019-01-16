cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

IF(WIN32)
   add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
ELSE(WIN32)
   add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
ENDIF(WIN32)

target_include_directories(DBoW2 PRIVATE ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})

target_link_libraries(DBoW2 ${OpenCV_LIBS})

install(TARGETS DBoW2 EXPORT DBoW2_TARGETS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include) #/DBoW2)
install(
    DIRECTORY DBoW2
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")
install(
    DIRECTORY DUtils
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")
