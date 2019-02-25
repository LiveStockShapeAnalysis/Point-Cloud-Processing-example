cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(boundary)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (boundary boundary.cpp)
target_link_libraries (boundary ${PCL_LIBRARIES})
 

