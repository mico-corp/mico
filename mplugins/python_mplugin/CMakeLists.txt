##---------------------------------------------------------------------------------------------------------------------
##  Python MICO plugin
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92) & Marco Montes Grova (a.k.a mgrova)
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
set(PLUGIN_NAME ${PROJECT_NAME}-python)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(boost-python-module-name "python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}" CACHE STRING "Boost::python module name, e.g. 'python-3.6'")

if (WIN32)
    find_package(Boost REQUIRED COMPONENTS ${boost-python-module-name})
elseif(UNIX)
    find_package(Boost REQUIRED COMPONENTS ${boost-python-module-name} numpy)
endif()


##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_PYTHON_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_PYTHON_HEADER_FILES "include/*.h" "include/*.inl")
  
SET(MICO_MODULES ${MICO_MODULES} python PARENT_SCOPE)
add_mplugin(  PLUGIN_NAME ${PLUGIN_NAME}
            PLUGIN_SOURCES ${MICO_PYTHON_SOURCE_FILES}
            PLUGIN_HEADERS  ${MICO_PYTHON_HEADER_FILES} 
            MICO_DEPS core
			HAS_RESOURCES)
  
##################################################
######    Loading 3rd party libraries.    ########
##################################################
    

find_package(Eigen3 REQUIRED)
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES Eigen3::Eigen)

find_package(OpenCV REQUIRED COMPONENTS core)
mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${OpenCV_INCLUDE_DIRS})
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${OpenCV_LIBS})
mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_OPENCV)
    
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES Python3::Python IS_PUBLIC)
if(WIN32)
    mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES "C:/vcpkg/packages/boost-python_x64-windows/lib/boost_numpy310-vc140-mt.lib" IS_PUBLIC)
elseif(UNIX)
    mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES Boost::numpy IS_PUBLIC)
endif()
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES "Boost::${boost-python-module-name}" IS_PUBLIC)
mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_PYTHON)

