cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(matrix_ransac)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (matrix_ransac matrix_ransac.cpp)
target_link_libraries (matrix_ransac ${PCL_LIBRARIES})
 

