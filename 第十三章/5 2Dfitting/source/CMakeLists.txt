cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(2dcurvefitting)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (2dcurvefitting 2dcurvefitting.cpp)
target_link_libraries (2dcurvefitting ${PCL_LIBRARIES})
 

