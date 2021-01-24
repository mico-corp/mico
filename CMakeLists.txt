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
project(mico VERSION 1.0.0 LANGUAGES C CXX)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

include(cmake/mico_functions.cmake)
include(cmake/mico_dependencies.cmake)

defineRootDir()
loadDefaultMicoDependencies()

if(WIN32)
    set(LIBRARY_MODE STATIC)
    # if(MSVC)
    #     add_compile_options(
    #         $<$<CONFIG:>:/MT> #---------|
    #         $<$<CONFIG:Debug>:/MTd> #---|-- Statically link the runtime libraries
    #         $<$<CONFIG:Release>:/MT> #--|
    #     )
    # endif()
elseif(UNIX)
    set(LIBRARY_MODE SHARED)
endif()

set(MICO_CLONE_ALL_DEPENDENCIES FALSE)
set(BUNDLE_COMPILATION ON)

option(BUILD_CAMERAS        "Set to build CAMERAS Module"       ON)
option(BUILD_VISUALIZERS    "Set to build VISUALIZERS Module"   ON)
option(BUILD_SLAM           "Set to build SLAM Module"          OFF)
option(BUILD_PYTHON         "Set to build Python Module"        OFF)
option(BUILD_DNN            "Set to build DNN Module"           OFF)
option(BUILD_ML             "Set to build ML Module"           ON)
option(BUILD_FASTCOM        "Set to build fastcom Module"       ON)
option(BUILD_ROBOTICS       "Set to build robotics Module"      OFF)
option(BUILD_ARDUINO        "Set to build arduino Module"       ON)

SET(MICO_MODULES "")

add_subdirectory(nodeeditor)
add_subdirectory(flow)
add_subdirectory(core_mplugin)
add_subdirectory(math_mplugin)

if(${BUILD_CAMERAS})
    add_subdirectory(cameras_mplugin)
endif()

if(${BUILD_SLAM})
    add_subdirectory(slam_mplugin)
endif()

if(${BUILD_VISUALIZERS})
    add_subdirectory(visualizers_mplugin)
endif()

if(${BUILD_DNN})
    add_subdirectory(dnn_mplugin)
endif()

if(${BUILD_ML})
    add_subdirectory(ml_mplugin)
endif()

if(${BUILD_PYTHON})
    add_subdirectory(python_mplugin)
endif()

if(${BUILD_FASTCOM})
    add_subdirectory(fastcom_mplugin)
endif()

if(${BUILD_ROBOTICS})
    add_subdirectory(robotics_mplugin)
endif()

if(${BUILD_ARDUINO})
    add_subdirectory(arduino_mplugin)
endif()

# Add custom comand for flow install of all selected modules
set(PLUGINS_DIR "")
if(WIN32)   
    set(PLUGINS_DIR "C:/.flow/plugins")
elseif(UNIX)
    set(PLUGINS_DIR $ENV{HOME}/.flow/plugins)
endif()

add_custom_target(flow_install
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/plugins 
                                     $ENV{HOME}/.flow/plugins
)



# ----------------------------------------------------------------------------
# Summary:
# ----------------------------------------------------------------------------
message(STATUS "==========================================================================")
message(STATUS "==========================================================================")
message(STATUS "")
message(STATUS "General configuration for mico ${mico_VERSION} ")
message(STATUS "    Machine:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "    CMake:"            ${CMAKE_VERSION})
message(STATUS "    CMake generator:"  ${CMAKE_GENERATOR})
message(STATUS "    CMake build tool:" ${CMAKE_BUILD_TOOL})
message(STATUS "")
message(STATUS "  C/C++:")
message(STATUS "    C++ Compiler:"           ${CMAKE_CXX_COMPILER})
message(STATUS "    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
message(STATUS "    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
message(STATUS "    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
message(STATUS "    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
message(STATUS "    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
message(STATUS "    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
message(STATUS "    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})
message(STATUS "")
message(STATUS "  mico modules: ${MICO_MODULES}")
message(STATUS "")
message(STATUS "  Install to:" "${CMAKE_INSTALL_PREFIX}")
message(STATUS "==========================================================================")
message(STATUS "")