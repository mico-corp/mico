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


macro(micoPrepareInstaller)
    
    # Custom target for packaging.
    if(MICO_USE_NSIS)
        set(CPACK_GENERATOR "NSIS")
    elseif(MICO_USE_DEB)
        set(CPACK_GENERATOR "DEB")
    else()
        set(CPACK_GENERATOR "ZIP")
    endif(MICO_USE_NSIS)

    include(InstallRequiredSystemLibraries)
    set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
    set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
    set(CPACK_PACKAGE_VERSION_PATCH "0")
    set(CPACK_PACKAGE_VENDOR "mico-corp")
    set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY ${PROJECT_NAME})
    if(WIN32)
        set(CPACK_PACKAGE_INSTALL_DIRECTORY "mico-corp\\mico")
        set(CPACK_IGNORE_FILES "\\.psd$;/CVS/;/\\.svn/;/\\.git/;\\.swp$;/CMakeLists.txt.user;\\.#;/#;\\.tar.gz$;/CMakeFiles/;CMakeCache.txt;\\.qm$;/build/;\\.diff$;.DS_Store'")
    else()
        set(CPACK_PACKAGE_INSTALL_DIRECTORY "mico-corp/mico")
        set(CPACK_IGNORE_FILES "/.psd$;/CVS/;/.svn/;/.git/;/.swp$;/CMakeLists.txt.user;/.#;/#;/.tar.gz$;/CMakeFiles/;CMakeCache.txt;/.qm$;/build/;/.diff$;.DS_Store'")
    endif()
    set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
    set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
    set(CPACK_SOURCE_GENERATOR "TGZ")
    set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
    set(CPACK_SOURCE_IGNORE_FILES ${CPACK_IGNORE_FILES})
    
    # Configure executables
    set(CPACK_PACKAGE_EXECUTABLES "flow_kids" "Flow for Kids")

    # Specific NSIS configuration
    if(MICO_USE_NSIS)
        set(CPACK_NSIS_MODIFY_PATH "OFF")
        set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/doc/mico.ico")
        set(CPACK_NSIS_MUI_UNIICON "${PROJECT_SOURCE_DIR}/doc/mico.ico")
        set(CPACK_NSIS_HELP_LINK "https://mico-corp.github.io/mico")
        set(CPACK_NSIS_URL_INFO_ABOUT "https://mico-corp.github.io/mico")
        # set(CPACK_NSIS_CONTACT ${APP_EMAIL})
        # Configure file with custom definitions for NSIS.

        configure_file(
            ${PROJECT_SOURCE_DIR}/NSIS.definitions.nsh.in
            ${CMAKE_CURRENT_BINARY_DIR}/NSIS.definitions.nsh )

        set(CPACK_NSIS_CREATE_ICONS_EXTRA
            "CreateShortCut '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Flow Kids.lnk' '$INSTDIR\\\\bin\\\\flow_kids.exe'"
        )

        set(CPACK_NSIS_DELETE_ICONS_EXTRA
            "Delete '$SMPROGRAMS\\\\$START_MENU\\\\Flow Kids.lnk'"
        )

        # More info https://www.mantidproject.org/NSIS_CPACK_Customisations.html
        # https://martinrotter.github.io/it-programming/2014/05/09/integrating-nsis-cmake/
        # https://cmake.cmake.narkive.com/pqtiaSbt/cpack-add-and-define-an-environment-variable-for-nsis
    
        set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/nsis_files ${CMAKE_MODULE_PATH})
        

        # Download plugin https://nsis.sourceforge.io/EnVar_plug-in
        if(WIN32 AND NOT EXISTS "C:/Program Files (x86)/NSIS/Plugins/amd64-unicode/EnVar.dll")
            MESSAGE(WARNING "EnVar pluging is not installed in your OS. Please, download and install it https://nsis.sourceforge.io/EnVar_plug-in to be able to generate NSIS packages")
        endif()
        # set(nsis_envar_plugin "https://nsis.sourceforge.io/mediawiki/images/7/7f/EnVar_plugin.zip")
        # file(DOWNLOAD ${nsis_envar_plugin} ${CMAKE_BINARY_DIR}/EnVar_plugin.zip)
        # file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/EnVar_plugin.zip DESTINATION "C:/Program Files (x86)/NSIS")

    endif()

    # Specific DEB configuration
    if(MICO_USE_DEB)
        SET(CPACK_GENERATOR "DEB")
        SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "mico-corp") #required

	    if(${TARGET_OS} STREQUAL "Raspbian")
            SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-all-dev, \
            libeigen3-dev,  \
            libflann-dev, \
            libopencv-dev, \
            libopencv-contrib-dev, \
            qt5-qmake, \
            libqt5opengl5-dev, \
            qtbase5-dev, \
            qttools5-dev-tools, \
            libgl1-mesa-dev, \
            freeglut3-dev, \
            libglew-dev")

	    else()

            set(LIST_DEPENDENCIES "libboost-all-dev, 
            libgraphviz-dev, 
            libeigen3-dev,  
            libflann-dev, 
            libopencv-dev, 
            libopencv-contrib-dev, 
            qt5-qmake, 
            qt5-default, 
            libqt5opengl5-dev, 
            qtbase5-dev, 
            qttools5-dev-tools, 
            qtmultimedia5-dev, 
            libusb-1.0-0-dev, 
            mesa-common-dev, 
            libgl1-mesa-dev, 
            freeglut3-dev, 
            libglew-dev")

            if(${BUILD_DVS})
                set(LIST_DEPENDENCIES "${LIST_DEPENDENCIES},
                                       dv-runtime-dev")
            endif()

            SET(CPACK_DEBIAN_PACKAGE_DEPENDS ${LIST_DEPENDENCIES})

	    endif()
    endif()


    # Call final CPACK configuration and prepare nsis file
    include(CPack)
endmacro(micoPrepareInstaller)
