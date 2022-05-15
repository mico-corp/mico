
##---------------------------------------------------------------------------------------------------------------------
##  FLOW
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

add_executable(MICO flow_kids.cpp)
target_include_directories(MICO PUBLIC ../include)
target_link_libraries(MICO LINK_PUBLIC flow)

if(${BUILD_PYTHON})
    # 666 Ugly workaround to load dynamically numpy libraries. For some reason, it fails finding python when loading numpy. The error is common, but
    # I all that I find is for applications using python and numpy directly, not a library that is loaded
    # by other app that does not have python I tried. The problem is stated here https://stackoverflow.com/a/56018943 ,
    # But the solution does not work in my case, probably because of the indirect usage of numpy inside of a
    # library, loaded dynamically by an app without python. Apart from this, the main file of flow_kids has been modified.
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    set(boost-python-module-name "python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}" CACHE STRING "Boost::python module name, e.g. 'python-3.6'")
    
    if (WIN32)
        find_package(Boost REQUIRED COMPONENTS ${boost-python-module-name})
    elseif(UNIX)
        find_package(Boost REQUIRED COMPONENTS ${boost-python-module-name} numpy)
    endif()


    target_link_libraries(MICO PRIVATE Python3::Python)
    if(WIN32)
        target_link_libraries(MICO PRIVATE "C:/vcpkg/packages/boost-python_x64-windows/lib/boost_numpy310-vc140-mt.lib")
    elseif(UNIX)
        target_link_libraries(MICO PRIVATE Boost::numpy)
    endif()
    target_compile_definitions(MICO PRIVATE -DINITIALIZE_PYTHON)
endif()



add_subdirectory(block_creator)

### Add test with input file
# find_package(fastcom REQUIRED)
# find_package(catkin COMPONENTS roscpp )#dvs_msgs)

# add_executable(speed_test speed_test.cpp)
# target_link_libraries(speed_test LINK_PRIVATE flow fastcom)
# target_link_libraries(speed_test LINK_PUBLIC ${catkin_LIBRARIES})
# target_include_directories(speed_test PUBLIC ${catkin_INCLUDE_DIRS})