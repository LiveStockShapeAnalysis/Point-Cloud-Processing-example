cmake_minimum_required( VERSION 2.8 )  
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )  
   
project( sample )  
add_executable( sample kinect2_grabber.h main.cpp )  
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "sample" )  
   
# Find Packages  
find_package( PCL 1.7.2 REQUIRED )  
find_package( KinectSDK2 REQUIRED )  
   
if( PCL_FOUND AND KinectSDK2_FOUND )  
  # Additional Include Directories  
  include_directories( ${PCL_INCLUDE_DIRS} )  
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )  
   
  # Preprocessor Definitions  
  add_definitions( ${PCL_DEFINITIONS} )  
   
  # Additional Library Directories  
  link_directories( ${PCL_LIBRARY_DIRS} )  
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )  
   
  # Additional Dependencies  
  target_link_libraries( sample ${PCL_LIBRARIES} )  
  target_link_libraries( sample ${KinectSDK2_LIBRARIES} )  
endif()  