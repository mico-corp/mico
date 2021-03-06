##---------------------------------------------------------------------------------------------------------------------
##  MICO Cameras wrapper plugin
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
set(PLUGIN_NAME ${PROJECT_NAME}-cameras)





##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_CAMERAS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_CAMERAS_HEADER_FILES "include/*.h" "include/*.inl")
 
SET(MICO_MODULES ${MICO_MODULES} cameras PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_CAMERAS_SOURCE_FILES}
				PLUGIN_HEADERS  ${MICO_CAMERAS_HEADER_FILES} 
				MICO_DEPS core
				HAS_RESOURCES)


##################################################
######    Loading 3rd party libraries.    ########
##################################################

if(${TARGET_OS} STREQUAL "Raspbian")
    mplugin_compile_definition(PLUGIN_NAME ${PLUGIN_NAME} IS_PUBLIC PLUGIN_DEFINITIONS MICO_IS_RASPBIAN)
endif()

find_package(OpenCV 4.2 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${OpenCV_INCLUDE_DIRS})
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${OpenCV_LIBS})

if(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "4")
	mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_OPENCV_4)
elseif(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "3")
	mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_OPENCV_3)
else()
	MESSAGE(FATAL_ERROR "Not possible to compile with OpenCV 2")
endif()


if(${TARGET_OS} STREQUAL "Raspbian")
	find_package(raspicam)
    mplugin_compile_definition(PLUGIN_NAME ${PLUGIN_NAME} IS_PUBLIC PLUGIN_DEFINITIONS MICO_IS_RASPBIAN)
	if(${raspicam_FOUND})
		mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${raspicam_CV_LIBS})
	endif()
endif()


