cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ply2pcd)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ply2pcd ply2pcd.cpp)
target_link_libraries (ply2pcd ${PCL_LIBRARIES})
 

