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


add_executable(benchmark_flow benchmark_flow.cpp)
target_link_libraries(benchmark_flow PRIVATE flow)

find_package(OpenCV 4.2 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
target_include_directories(benchmark_flow PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(benchmark_flow PRIVATE ${OpenCV_LIBS})

find_package(catkin COMPONENTS roscpp)
if(${catkin_FOUND})
    add_executable(benchmark_ros benchmark_ros.cpp)
    target_compile_features(benchmark_ros PRIVATE cxx_std_20)
    target_link_libraries(benchmark_ros PRIVATE ${catkin_LIBRARIES})
    target_include_directories(benchmark_ros PRIVATE ${catkin_INCLUDE_DIRS})
    
endif()