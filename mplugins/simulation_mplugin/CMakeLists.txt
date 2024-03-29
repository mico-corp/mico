##---------------------------------------------------------------------------------------------------------------------
##  MICO math wrapper plugin
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
set(PLUGIN_NAME ${PROJECT_NAME}-simulation)


##################################################
######    Configure project               ########
##################################################

##################################################
######    Loading 3rd party libraries.    ########
##################################################

set(EXTRA_INCLUDES "")
set(EXTRA_LIBRARIES "")
set(EXTRA_COMPILE_DEFS "")

##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_SIMULATION_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_SIMULATION_HEADER_FILES "include/*.h" "include/*.inl")
 
SET(MICO_MODULES ${MICO_MODULES} simulation PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_SIMULATION_SOURCE_FILES}
				PLUGIN_HEADERS  ${MICO_SIMULATION_HEADER_FILES} 
				MICO_DEPS core
				HAS_RESOURCES
				)

##################################################
######    Loading 3rd party libraries.    ########
##################################################
