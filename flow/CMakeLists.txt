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

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
PROJECT(flow VERSION 0.1.0)

if(WIN32)
	set(Boost_USE_STATIC_LIBS ON)
endif()

#########################################
######			General config	   ######
#########################################
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

option(BUILD_EXAMPLES   "Compile examples"              OFF)
option(BUILD_TESTS      "Prepare tests"                 ON)

##################################################
######    Library.                        ########
##################################################
file(GLOB_RECURSE FLOW_HEADERS "./include/*.h" "./include/*.inl")
file(GLOB_RECURSE FLOW_SOURCES "./src/*.cpp")

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

add_library(${PROJECT_NAME} ${LIBRARY_MODE} ${FLOW_HEADERS} ${FLOW_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

##################################################
######    Loading 3rd party libraries.    ########
##################################################

if(UNIX)
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC pthread stdc++fs)
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem program_options exception)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Boost::boost 
                                                  Boost::system 
                                                  Boost::filesystem 
                                                  Boost::program_options 
                                                  Boost::exception)
                                                  
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Widgets)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    FLOW_LIB_SHARED
  PRIVATE
    FLOW_EXPORTS
)

if(WIN32)
    find_package(dlfcn-win32 REQUIRED)
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC dlfcn-win32::dl)
endif()


##################################################
######    Test.                           ########
##################################################
if(${BUILD_TESTS})
  find_package(GTest CONFIG)
  if(${GTest_FOUND})
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()

##################################################
######    Examples.                       ########
##################################################
#if(${BUILD_EXAMPLES})
#  add_subdirectory(examples)
#endif()

##################################################
######    Installation.                   ########
##################################################
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME} 
        EXPORT ${PROJECT_NAME}-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
        )

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# This installs the include folder
install(  DIRECTORY include/ 
          DESTINATION include 
          FILES_MATCHING PATTERN "*.h")
          
install(  DIRECTORY include/ 
          DESTINATION include 
          FILES_MATCHING PATTERN "*.inl")


export(TARGETS flow NAMESPACE flow:: FILE flow.cmake)

# This generates flow-targets.cmake
install(EXPORT ${PROJECT_NAME}-targets
    FILE "${PROJECT_NAME}-targets.cmake"
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
    )

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    @ONLY
  )
  
install(FILES 
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
        )


if(WIN32)
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
    )

    file(GLOB flow_dlls ${CMAKE_BINARY_DIR}/bin/*.dll)
    install(FILES ${flow_dlls}
        DESTINATION bin
    )

    FILE(GLOB plugins_dirs ${CMAKE_BINARY_DIR}/bin/plugins/*)
      install(DIRECTORY ${plugins_dirs} 
          DESTINATION bin
      )

    # ucrtbase.dll y VCRUNTIME140D make it automatic please.... 
    install(    FILES 
                "C:/Program Files (x86)/Windows Kits/10/bin/10.0.19041.0/x64/ucrt/ucrtbased.dll"
                "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.38.33130/debug_nonredist/x64/Microsoft.VC143.DebugCRT/vcruntime140d.dll"
                "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.38.33130/debug_nonredist/x64/Microsoft.VC143.DebugCRT/vcruntime140_1d.dll"
                "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.38.33130/debug_nonredist/x64/Microsoft.VC143.DebugCRT/msvcp140d.dll"
                "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.38.33130/debug_nonredist/x64/Microsoft.VC143.DebugCRT/msvcp140_1d.dll"
                "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.38.33130/debug_nonredist/x64/Microsoft.VC143.DebugCRT/concrt140d.dll"
                DESTINATION bin )
    
elseif(UNIX)
  install(TARGETS ${PROJECT_NAME}
      LIBRARY DESTINATION bin
  )
endif()
