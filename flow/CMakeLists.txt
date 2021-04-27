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
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

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

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 


##################################################
####  CPACK deb generation configuration    ######
##################################################
if(${PREPARE_PACKAGE_PPA})
  set(UPLOAD_PPA "ppa:bardo91/mico")
  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Pablo Ramon Soria <pabramsor@gmail.com>") #required
  set(CPACK_DEBIAN_PACKAGE_NAME ${PLUGIN_NAME})
  
  include(CPack)
  include(cmake/templates/prepare_package_ppa.cmake)
endif()









