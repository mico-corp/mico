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

macro(micoInstallFlann _installDir)
    ##Check if already installed
    if(NOT EXISTS ${_installDir}/dependencies/include/flann)
        if(UNIX)
            execute_process(COMMAND  sudo apt-get install -y libflann-dev)
        elseif(WIN32)
            execute_process(COMMAND  ${CMAKE_SOURCE_DIR}/cmake/dependencies/win_install_impl/installFlann.bat ${_installDir}/tmp ${_installDir}/dependencies)
        else()
            message(FATAL_ERROR "Cannot build for current OS")
        endif()
    endif()
    
    ##find_package(flann HINTS ${_installDir}/dependencies REQUIRED)    
    if(EXISTS ${_installDir}/dependencies/include/flann)
        set(FLANN_INCLUDE_DIR ${_installDir}/dependencies/include)
        set(FLANN_LIBRARY_DIR ${_installDir}/dependencies/lib)
        set(FLANN_LIBRARIES flann.lib flann_cpp_s.lib flann_s.lib)
    else()
        message(FATAL_ERROR "Flann was not properly installed look for help!")
    endif()
    
endmacro(micoInstallFlann)
