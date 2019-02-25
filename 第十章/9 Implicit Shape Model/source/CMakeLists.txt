cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(implicit_shape_model)

find_package(PCL 1.5 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (implicit_shape_model_training implicit_shape_model_training.cpp)
target_link_libraries (implicit_shape_model_training ${PCL_LIBRARIES})
add_executable (implicit_shape_model_classification implicit_shape_model_classification.cpp)
target_link_libraries (implicit_shape_model_classification ${PCL_LIBRARIES})