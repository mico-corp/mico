##---------------------------------------------------------------------------------------------------------------------
##  MICO TEMPLATE plugin
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

include(ExternalProject)
include(cmake/dependencies/mico_install_boost.cmake)
include(cmake/dependencies/mico_install_eigen3.cmake)
include(cmake/dependencies/mico_install_flann.cmake)
include(cmake/dependencies/mico_install_vtk.cmake)
include(cmake/dependencies/mico_install_pcl.cmake)
include(cmake/dependencies/mico_install_opencv.cmake)
include(cmake/dependencies/mico_install_qt5.cmake)
include(cmake/dependencies/mico_install_dlfcn.cmake)
include(cmake/dependencies/mico_install_pybind11.cmake)
include(cmake/dependencies/mico_install_fastcom.cmake)

macro(defineRootDir)
    if(WIN32)
        set(MICO_ROOT_DIR "c:/users/$ENV{USERNAME}/mico/")
    elseif(UNIX)
        set(MICO_ROOT_DIR "/home/$ENV{USER}/mico/")        
    else()
        set(MICO_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR/mico/")
    endif()

    if(NOT DEFINED CMAKE_INSTALL_PREFIX)
        set(CMAKE_INSTALL_PREFIX ${MICO_ROOT_DIR})
    endif() 

endmacro(defineRootDir)

macro(loadDefaultMicoDependencies)
    if(${MICO_INSTALL_ALL_DEPENDENCIES})
        defineRootDir()

        micoInstallBoost(${MICO_ROOT_DIR})
        micoInstallEigen(${MICO_ROOT_DIR})
        micoInstallFlann(${MICO_ROOT_DIR})
        micoInstallVtk(${MICO_ROOT_DIR})
        micoInstallPcl(${MICO_ROOT_DIR})
        micoInstallOpencv(${MICO_ROOT_DIR})
        micoInstallQt5(${MICO_ROOT_DIR})
        micoInstallDlfcn(${MICO_ROOT_DIR})
        
        if(${BUILD_FASTCOM})
            micoInstallFastcom(${MICO_ROOT_DIR})
        endif()

        if(${BUILD_PYTHON})
            micoInstallPybind11(${MICO_ROOT_DIR})
        endif()

        # Install doxygen 666
    else()
        ## Find Qt5
        find_package(Qt5 5.12 COMPONENTS Core Widgets REQUIRED)
        ## Boost
        find_package(Boost REQUIRED)
        ## Find Eigen
        find_package(Eigen3 REQUIRED)
        ## Find OpenCV
        find_package(OpenCV REQUIRED)
        ## Find PCL
        find_package(PCL  QUIET REQUIRED)
        ## Find Doxygen
        find_package(Doxygen)

    endif()
endmacro(loadDefaultMicoDependencies)
