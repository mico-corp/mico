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
	add_compile_options("/std:c++17")
endif()

add_subdirectory(thirdparty/nodeeditor)

#########################################
######			General config	   ######
#########################################
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

option(BUILD_EXAMPLES   "Compile examples"              OFF)
option(BUILD_TESTS      "Prepare tests"                 OFF)
option(BUILD_TOOLS      "Build tools"                   ON)

set(CMAKE_AUTOMOC ON)

##################################################
######    Library.                        ########
##################################################
file(GLOB_RECURSE FLOW_HEADERS "./include/*.h" "./include/*.inl")
file(GLOB_RECURSE FLOW_SOURCES "./src/*.cpp")

add_library(${PROJECT_NAME}  ${LIBRARY_MODE} ${FLOW_HEADERS} ${FLOW_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

##################################################
######    Loading 3rd party libraries.    ########
##################################################

if(UNIX)
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC pthread X11 stdc++fs)
endif()

if(${BUNDLE_COMPILATION})
  target_link_libraries(${PROJECT_NAME} LINK_PUBLIC NodeEditor::nodes)
  target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_QTNODEEDITOR)
else()
  find_package(NodeEditor)
  if(${NodeEditor_FOUND})
    target_include_directories(${PROJECT_NAME} PUBLIC ${NodeEditor_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${NodeEditor_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_QTNODEEDITOR)
  endif()
endif()

find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem program_options exception)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Boost::boost 
                                                  Boost::system 
                                                  Boost::filesystem 
                                                  Boost::program_options 
                                                  Boost::exception)

if(WIN32)
    find_package(dlfcn-win32 REQUIRED)
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC dlfcn-win32::dl)
endif()


target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    FLOW_LIB_SHARED
  PRIVATE
    FLOW_EXPORTS
)

target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Widgets)

add_custom_target(flow_resources ALL
                COMMAND ${CMAKE_COMMAND} -E copy_directory 	${CMAKE_CURRENT_SOURCE_DIR}/resources 
                ${CMAKE_BINARY_DIR}/plugins/resources
)

##################################################
######    Test.                           ########
##################################################
if(${BUILD_TESTS})
  find_package(GTest)
  if(${GTest_FOUND})
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()

##################################################
######    Examples.                       ########
##################################################
if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

##################################################
######    Tools.                          ########
##################################################
if(${BUILD_TOOLS})
  add_subdirectory(tools)
endif()

##################################################
######    Installation.                   ########
##################################################
include(CMakePackageConfigHelpers)

install(TARGETS nodes ${PROJECT_NAME} 
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

install(  DIRECTORY resources/ 
          DESTINATION bin/mplugins/resources )
          

export(TARGETS nodes flow NAMESPACE flow:: FILE flow.cmake)

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

if(${BUILD_TOOLS})
  install(TARGETS flow_kids
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION bin
    RUNTIME DESTINATION bin
  )
endif()


# Additional library requirements
if(WIN32)
    # Qt
    install(    FILES 
                ${Qt5_DIR}/../../../bin/Qt5Core.dll
                ${Qt5_DIR}/../../../bin/Qt5Gui.dll
                ${Qt5_DIR}/../../../bin/Qt5PrintSupport.dll
                ${Qt5_DIR}/../../../bin/Qt5Svg.dll
                ${Qt5_DIR}/../../../bin/Qt5Widgets.dll
                DESTINATION bin )
    
    install(    DIRECTORY ${Qt5_DIR}/../../../plugins/platforms
                DESTINATION bin )

    install(    DIRECTORY ${Qt5_DIR}/../../../plugins/iconengines
                DESTINATION bin )

    find_package(OpenCV)
    # OpenCV
    install(    FILES 
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_aruco420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_calib3d420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_core420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_dnn420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_features2d420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_flann420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_highgui420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_imgproc420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_imgcodecs420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_objdetect420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_videoio420.dll
                ${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_xfeatures2d420.dll
                DESTINATION bin )
    
    # dlfcn
    install(    FILES 
                ${dlfcn-win32_DIR}/../../bin/dl.dll
                DESTINATION bin )
          
    # Freeglut
    install(    FILES 
                ${FreeGLUT_DIR}/../../../bin/freeglut.dll
                DESTINATION bin )

    # ucrtbase.dll y VCRUNTIME140D make it automatic please.... 
    install(    FILES 
                "C:/Program Files (x86)/Windows Kits/10/bin/10.0.18362.0/x64/ucrt/ucrtbased.dll"
                "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Redist/MSVC/14.28.29325/debug_nonredist/x64/Microsoft.VC142.DebugCRT/vcruntime140d.dll"
                DESTINATION bin )
    
     
endif()