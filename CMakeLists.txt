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

set(BUNDLE_COMPILATION ON)

option(BUILD_CAMERAS        "Set to build CAMERAS Module"       ON)
option(BUILD_VISUALIZERS    "Set to build VISUALIZERS Module"   ON)
option(BUILD_SLAM           "Set to build SLAM Module"          ON)
option(BUILD_PYTHON         "Set to build Python Module"        ON)
option(BUILD_DNN            "Set to build DNN Module"           OFF)

SET(MICO_MODULES core)

add_subdirectory(nodeeditor)
add_subdirectory(flow)
add_subdirectory(core_mplugin)

list(APPEND MICO_MODULES math)
add_subdirectory(math_mplugin)

if(${BUILD_CAMERAS})
    list(APPEND MICO_MODULES cameras)
    add_subdirectory(cameras_mplugin)
endif()

if(${BUILD_SLAM})
    list(APPEND MICO_MODULES slam)
    add_subdirectory(slam_mplugin)
endif()

if(${BUILD_VISUALIZERS})
    list(APPEND MICO_MODULES visualizers)
    add_subdirectory(visualizers_mplugin)
endif()

if(${BUILD_DNN})
    list(APPEND MICO_MODULES dnn)
    add_subdirectory(dnn_mplugin)
endif()

if(${BUILD_PYTHON})
    list(APPEND MICO_MODULES python)
    add_subdirectory(python_mplugin)
endif()

# Add custom comand for flow install of all selected modules
add_custom_target(flow_install
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/plugins 
                                     $ENV{HOME}/.flow/plugins
)
