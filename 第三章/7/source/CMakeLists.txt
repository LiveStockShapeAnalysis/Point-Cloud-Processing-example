cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2ply)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd2ply pcd2ply.cpp)
target_link_libraries (pcd2ply ${PCL_LIBRARIES})
 

