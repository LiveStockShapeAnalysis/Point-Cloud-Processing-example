cmake_minimum_required(VERSION 2.8)
include (GenerateExportHeader)
project(RGlib)

FIND_PACKAGE(Qt4 REQUIRED)
find_package(PCL 1.7 REQUIRED)



SET (LIB_TYPE SHARED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/header")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DNOMINMAX)

set(INCS
	"header/stdafx.h"
)

set(SRCS
	src/IinitalRG.cpp
	src/FineRG.cpp
)


set(MOC_IinitalRG_INC "header/IinitalRG.h")
set(MOC_FineRG_INC "header/FineRG.h")

INCLUDE(${QT_USE_FILE})
qt4_wrap_cpp(MOC_IinitalRG_SRC "${MOC_IinitalRG_INC}")
qt4_wrap_cpp(MOC_FineRG_SRC "${MOC_FineRG_INC}")

list(APPEND ADDITIONAL_LIBS ${QT_LIBRARIES})
list(APPEND SRCS "${MOC_IinitalRG_SRC}" "${MOC_FineRG_SRC}")
list(APPEND INCS "${MOC_IinitalRG_INC}" "${MOC_FineRG_INC}")



# Create a target for the library
ADD_LIBRARY(RGlib ${LIB_TYPE} ${SRCS} ${INCS})
target_link_libraries(RGlib ${QT_LIBRARIES} ${PCL_LIBRARIES})

GENERATE_EXPORT_HEADER( RGlib
             BASE_NAME RGlib
             EXPORT_MACRO_NAME RGlib_EXPORT
             EXPORT_FILE_NAME RGlib_Export.h)

set (test_SRCS   main.cpp
		Registration.cpp )
set (test_INCS	 Registration.h)
set (test_FORMS  MainWindow.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${test_INCS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${test_FORMS})


ADD_EXECUTABLE  (testrg ${test_SRCS}
								${test_INCS}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (testrg ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} RGlib)