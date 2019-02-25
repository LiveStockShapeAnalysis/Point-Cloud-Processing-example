cmake_minimum_required(VERSION 2.8)
include (GenerateExportHeader)
project(Lslib)

FIND_PACKAGE(Qt4 REQUIRED)
find_package(PCL 1.7 REQUIRED)



SET (LIB_TYPE SHARED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/third" "${CMAKE_CURRENT_SOURCE_DIR}/header")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DNOMINMAX)

set(INCS
	"header/stdafx.h"
	"third/InterfaceLLT_2.h"
	"third/DllLoader.h"
	"third/scanControlDataTypes.h"
)

set(SRCS
	src/LineScaner.cpp
	third/DllLoader.cpp
	third/InterfaceLLT_2.cpp
)


set(MOC_LINESCANER_INC "header/LineScaner.h")

INCLUDE(${QT_USE_FILE})
qt4_wrap_cpp(MOC_LINESCANER_SRC "${MOC_LINESCANER_INC}")

list(APPEND ADDITIONAL_LIBS ${QT_LIBRARIES})
list(APPEND SRCS "${MOC_LINESCANER_SRC}")
list(APPEND INCS "${MOC_LINESCANER_INC}")

# Create a target for the library
ADD_LIBRARY(LsLib ${LIB_TYPE} ${SRCS} ${INCS})
target_link_libraries(LsLib ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_subdirectory(sourceio)
add_subdirectory(sourceads)

GENERATE_EXPORT_HEADER( LsLib
             BASE_NAME LsLib
             EXPORT_MACRO_NAME LsLib_EXPORT
             EXPORT_FILE_NAME LsLib_Export.h)

find_package (VTK REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/header" "${CMAKE_CURRENT_SOURCE_DIR}/sourceio/header" "${CMAKE_CURRENT_SOURCE_DIR}/sourceads/header" "${CMAKE_CURRENT_SOURCE_DIR}/sourceads/third")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/sourceads/third")
set (test_SRCS   test/src/main.cpp
		test/src/MYscaner.cpp )
set (test_INCS	 test/header/MYscaner.h)
set (test_FORMS  test/resource/mainUI.ui)
set (test_RSRC test/resource/resource.qrc)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${test_INCS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${test_FORMS})
QT4_ADD_RESOURCES(RESOURCES_SRCS ${test_RSRC})

ADD_EXECUTABLE  (MIYI_capturing ${test_SRCS}
								${test_INCS}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC}
								${RESOURCES_SRCS})

TARGET_LINK_LIBRARIES (MIYI_capturing ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} LsLib IOLib AdsLib)

