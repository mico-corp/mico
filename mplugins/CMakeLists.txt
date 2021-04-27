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

if(${BUILD_SIMULATION})
	add_subdirectory(simulation_mplugin)
endif()

if(${BUILD_AR})
	add_subdirectory(ar_mplugin)
endif()

if(${BUILD_DVS})
	add_subdirectory(dvs_mplugin)
endif()

if(${BUILD_BENCHMARK})
	add_subdirectory(benchmark)
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
