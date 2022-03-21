##---------------------------------------------------------------------------------------------------------------------
##  Visualizers MICO plugin
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(mico VERSION 1.0 LANGUAGES C CXX)
set(PLUGIN_NAME ${PROJECT_NAME}-visualizers)

set(HAS_MICO_SLAM FALSE)
set(HAS_MICO_DNN FALSE)

if (TARGET mico-slam)
  set(HAS_MICO_SLAM TRUE)
endif()

if (TARGET mico-dnn)
  set(HAS_MICO_DNN TRUE)
endif()

##################################################
###### Create project and configure ALIAS ########
##################################################
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

file(GLOB_RECURSE MICO_VISUALIZERS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_VISUALIZERS_HEADER_FILES "include/*.h" "include/*.inl")

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
qt5_wrap_cpp(MOC_FILES include/mico/visualizers/qcustomplot.h)

##################################################
###### Create project and configure ALIAS ########
##################################################
set(MICO_DEPS "core")
set(EXTRA_DEFINITIONS "")
if(${HAS_MICO_SLAM})
  set(MICO_DEPS ${MICO_DEPS} slam)
  set(EXTRA_DEFINITIONS ${EXTRA_DEFINITIONS} HAS_MICO_SLAM)
endif()

if(${HAS_MICO_DNN})
  set(MICO_DEPS ${MICO_DEPS} dnn)
  set(EXTRA_DEFINITIONS ${EXTRA_DEFINITIONS} HAS_MICO_DNN)
endif()

SET(MICO_MODULES ${MICO_MODULES} visualizers PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_VISUALIZERS_SOURCE_FILES} ${MOC_FILES}
                PLUGIN_HEADERS  ${MICO_VISUALIZERS_HEADER_FILES} 
				HAS_RESOURCES
                MICO_DEPS ${MICO_DEPS})


mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS ${EXTRA_DEFINITIONS})
mplugin_compile_definition(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS QCUSTOMPLOT_COMPILE_LIBRARY IS_PRIVATE)

##################################################
######    Loading 3rd party libraries.    ########
##################################################

find_package(Pangolin)
if(${Pangolin_FOUND})
  mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${Pangolin_INCLUDE_DIRS})
  mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${Pangolin_LIBRARIES})
  mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS "MICO_HAS_PANGOLIN")
endif()

find_package(OpenCV REQUIRED)
mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${OpenCV_INCLUDE_DIRS})
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${OpenCV_LIBS})
mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_OPENCV)

find_package(PCL  QUIET REQUIRED)
mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${PCL_INCLUDE_DIRS})
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${PCL_LIBRARIES})
mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS ENABLE_PCL DISABLE_PCAP DISABLE_PNG DISABLE_LIBUSB_1_0)


mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES Qt5::Core Qt5::Widgets)

