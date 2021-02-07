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

macro(micoInstallPybind11 _installDir)

    ## Check that python is already installed and the additional components.
    if(WIN32)
        set(Python3_ROOT_DIR "C:/Program Files/Python39")
    endif()
    if(UNIX)
        execute_process(COMMAND sudo apt-get install -y python3-dev python3-pip)
	execute_process(COMMAND pip3 install numpy opencv-python)
    endif()

    find_package(Python3 QUIET COMPONENTS Interpreter Development NumPy REQUIRED)
    
    ##Check if already installed
    if(UNIX)
        execute_process(COMMAND  bash ${CMAKE_SOURCE_DIR}/cmake/dependencies/unix_install_impl/installPybind11.sh ${_installDir}/tmp ${_installDir}/dependencies)
    elseif(WIN32)
        if(NOT EXISTS ${_installDir}/dependencies/include/pybind11)
            execute_process(COMMAND  ${CMAKE_SOURCE_DIR}/cmake/dependencies/win_install_impl/installPybind11.bat ${_installDir}/tmp ${_installDir}/dependencies)
        endif()
    else()
        message(FATAL_ERROR "Cannot build for current OS")
    endif()
    
    find_package(pybind11 HINTS ${_installDir}/dependencies/cmake REQUIRED)    
endmacro(micoInstallPybind11)
