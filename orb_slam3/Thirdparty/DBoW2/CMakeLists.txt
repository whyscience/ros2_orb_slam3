#cmake_minimum_required(VERSION 3.8)
#project(DBoW2)

#find_package(OpenCV 4.2 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin
#set(DBOW2_ROOT_DIR ${PROJECT_SOURCE_DIR})

set (DBOW2_ROOT_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2")

set(HDRS_DBOW2
        ${DBOW2_ROOT_DIR}/DBoW2/BowVector.h
        ${DBOW2_ROOT_DIR}/DBoW2/FORB.h
        ${DBOW2_ROOT_DIR}/DBoW2/FClass.h
        ${DBOW2_ROOT_DIR}/DBoW2/FeatureVector.h
        ${DBOW2_ROOT_DIR}/DBoW2/ScoringObject.h
        ${DBOW2_ROOT_DIR}/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
        ${DBOW2_ROOT_DIR}/DBoW2/BowVector.cpp
        ${DBOW2_ROOT_DIR}/DBoW2/FORB.cpp
        ${DBOW2_ROOT_DIR}/DBoW2/FeatureVector.cpp
        ${DBOW2_ROOT_DIR}/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
        ${DBOW2_ROOT_DIR}/DUtils/Random.h
        ${DBOW2_ROOT_DIR}/DUtils/Timestamp.h)
set(SRCS_DUTILS
        ${DBOW2_ROOT_DIR}/DUtils/Random.cpp
        ${DBOW2_ROOT_DIR}/DUtils/Timestamp.cpp)

# set(LIBRARY_OUTPUT_PATH ${DBOW2_ROOT_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${DBOW2_ROOT_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

