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
    find_package(Python3 QUIET COMPONENTS Interpreter Development NumPy REQUIRED)
    
    ##Check if already installed
    if(NOT EXISTS ${_installDir}/dependencies/include/pybind11)
        if(UNIX)
            message(FATAL_ERROR "auto install not prepared in unix systems")
        elseif(WIN32)
            execute_process(COMMAND  ${CMAKE_SOURCE_DIR}/cmake/dependencies/win_install_impl/installPybind11.bat ${_installDir}/tmp ${_installDir}/dependencies)
        else()
            message(FATAL_ERROR "Cannot build for current OS")
        endif()
    endif()
    
    find_package(pybind11 HINTS ${_installDir}/dependencies/cmake REQUIRED)    
endmacro(micoInstallPybind11)
