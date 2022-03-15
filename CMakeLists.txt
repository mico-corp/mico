##---------------------------------------------------------------------------------------------------------------------
##  MICO 
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
project(mico VERSION 1.1.0 LANGUAGES C CXX)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

include(cmake/mico_functions.cmake)
include(cmake/mico_prepare_installer.cmake)

detect_OS()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/package)

if(WIN32)
    set(LIBRARY_MODE SHARED)
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

if(WIN32)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

option(BUILD_CAMERAS            "Set to build CAMERAS Module"       ON)
option(BUILD_VISUALIZERS        "Set to build VISUALIZERS Module"   ON)
option(BUILD_PYTHON             "Set to build Python Module"        OFF)
option(BUILD_ML                 "Set to build ML Module"            ON)
option(BUILD_IMGPROC            "Set to build imgproc Module"       ON)
option(BUILD_FASTCOM            "Set to build fastcom Module"       OFF)
option(BUILD_ROBOTICS           "Set to build robotics Module"      OFF)
option(BUILD_ARDUINO            "Set to build arduino Module"       ON)
option(BUILD_SIMULATION         "Set to build simulation Module"    OFF)
option(BUILD_AR                 "Set to build AR Module"            ON)
option(BUILD_DVS                "Set to build DSV Module"           OFF)

option(BUILD_BENCHMARK          "Set to build Benchmarks"           OFF)

option(MICO_PREPARE_INSTALLER   "Set to prepare installer" ON)
if(WIN32)
    option(MICO_USE_NSIS            "Use NSIS generator to produce installer" ON)
elseif(UNIX)
    option(MICO_USE_DEB             "Use DEB generator to produce installer" ON)
endif()


SET(MICO_MODULES "")

defineRootDir()

add_subdirectory(flow)
add_subdirectory(mplugins)
    
if(${MICO_PREPARE_INSTALLER})
    micoPrepareInstaller()
endif()


##################################################
######    Documentation.                  ########
##################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_mico
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)



# ----------------------------------------------------------------------------
# Summary:
# ----------------------------------------------------------------------------
message(STATUS "==========================================================================")
message(STATUS "==========================================================================")
message(STATUS "")
message(STATUS "General configuration for mico ${mico_VERSION} ")
message(STATUS "    Machine:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "    Target OS: "            ${TARGET_OS})
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
message(STATUS "  Install to: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "==========================================================================")
message(STATUS "")
