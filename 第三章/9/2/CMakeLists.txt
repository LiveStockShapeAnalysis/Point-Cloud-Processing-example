cmake_minimum_required(VERSION 2.8.3)
project(myviz)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myrviz
#  CATKIN_DEPENDS roscpp rospy rviz std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/myrviz.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/myrviz.cpp
  src/main.cpp
  ${MOC_FILES}
)

## Add the "myrviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(myrviz ${SOURCE_FILES})

## Link the myrviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(myrviz ${QT_LIBRARIES} default_plugin ${catkin_LIBRARIES})
## END_TUTORIAL

## Install
install(TARGETS myrviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
