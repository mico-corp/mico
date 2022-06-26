##---------------------------------------------------------------------------------------------------------------------
##  MICO Audio plugin
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
set(PLUGIN_NAME ${PROJECT_NAME}-audio)

##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_AUDIO_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_AUDIO_HEADER_FILES "include/*.h" "include/*.inl")
set(LIBSTD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libstdaudio/include)
 
include_directories(${LIBSTD_INCLUDE_DIR})

SET(MICO_MODULES ${MICO_MODULES} audio PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_AUDIO_SOURCE_FILES}
				PLUGIN_HEADERS  ${MICO_AUDIO_HEADER_FILES} 
				MICO_DEPS core
				HAS_RESOURCES)

if(WIN32)
	find_package(FFTW3 CONFIG REQUIRED)
	find_package(FFTW3f CONFIG REQUIRED)
	find_package(FFTW3l CONFIG REQUIRED)
	mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES FFTW3::fftw3 FFTW3::fftw3f FFTW3::fftw3l)
else()
	find_package(FFTW REQUIRED)
	mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${FFTW_LIBRARIES})
	mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${FFTW_INCLUDE_DIRS})
endif()


##################################################
######    Loading 3rd party libraries.    ########
##################################################



