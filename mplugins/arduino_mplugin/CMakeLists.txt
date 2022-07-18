##---------------------------------------------------------------------------------------------------------------------
##  MICO arduino wrapper plugin
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
cmake_minimum_required (VERSION 3.21 FATAL_ERROR)
project(mico VERSION 1.0 LANGUAGES C CXX)
set(PLUGIN_NAME ${PROJECT_NAME}-arduino)

##################################################
###### Create project and configure ALIAS ########
##################################################

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
file(GLOB_RECURSE MICO_ARDUINO_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_ARDUINO_HEADER_FILES "include/*.h" "include/*.inl")

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
qt5_wrap_cpp(MICO_ARDUINO_SOURCE_FILES_MOC "include/mico/arduino/flow/JoyPad.h")

SET(MICO_MODULES ${MICO_MODULES} arduino PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_ARDUINO_SOURCE_FILES} ${MICO_ARDUINO_SOURCE_FILES_MOC}
                PLUGIN_HEADERS  ${MICO_ARDUINO_HEADER_FILES} 
				MICO_DEPS core
				HAS_RESOURCES)


##################################################
######    Loading 3rd party libraries.    ########
##################################################

find_package(Eigen3 REQUIRED)
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES Eigen3::Eigen)
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES flow_qnodes)

if(${TARGET_OS} STREQUAL "Raspbian")
    mplugin_compile_definition(PLUGIN_NAME ${PLUGIN_NAME} IS_PUBLIC PLUGIN_DEFINITIONS MICO_IS_RASPBIAN)
    mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES pigpio)
endif()
##################################################
######    Library.                        ########
##################################################
#add_definitions(-fopenmp)
