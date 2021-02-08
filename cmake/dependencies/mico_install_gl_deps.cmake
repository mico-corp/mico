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

macro(micoInstallGl_deps _installDir)
    ##Check if already installed
    if(UNIX)
        MESSAGE(FATAL_ERROR "Not ready in ubuntu yet")
    elseif(WIN32)
        if(NOT EXISTS ${_installDir}/dependencies/include/GL)
            execute_process(COMMAND  ${CMAKE_SOURCE_DIR}/cmake/dependencies/win_install_impl/installGL_deps.bat ${_installDir}/tmp ${_installDir}/dependencies)
        endif()
    else()
        message(FATAL_ERROR "Cannot build for current OS")
    endif()
    
#    find_package(vtk HINTS ${_installDir}/dependencies REQUIRED)    
endmacro(micoInstallGl_deps)