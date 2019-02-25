cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(las2pcd)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/libLAS/include")

link_directories(${PCL_LIBRARY_DIRS})
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/libLAS/lib")
add_definitions(${PCL_DEFINITIONS})
		
add_executable (las2pcd las2pcd.cpp)
target_link_libraries (las2pcd ${PCL_LIBRARIES})
target_link_libraries (las2pcd "${CMAKE_CURRENT_SOURCE_DIR}/libLAS/lib/liblas.lib")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(DLL_LIBLAS "${CMAKE_CURRENT_SOURCE_DIR}/libLAS/dll/liblas.dll" "${CMAKE_CURRENT_SOURCE_DIR}/libLAS/dll/gdal111.dll" "${CMAKE_CURRENT_SOURCE_DIR}/libLAS/dll/geotiff.dll" "${CMAKE_CURRENT_SOURCE_DIR}/libLAS/dll/libtiff.dll")

file(COPY ${DLL_LIBLAS} DESTINATION ${EXECUTABLE_OUTPUT_PATH}/release)
file(COPY ${DLL_LIBLAS} DESTINATION ${EXECUTABLE_OUTPUT_PATH}/debug)