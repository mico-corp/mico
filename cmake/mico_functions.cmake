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


macro(add_mplugin)
    set(options HAS_RESOURCES)
    set(oneValueArgs PLUGIN_NAME)
    set(multiValueArgs PLUGIN_SOURCES PLUGIN_HEADERS MICO_DEPS)
    cmake_parse_arguments(IN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    set(MICO_PLUGIN_INCLUDES "")
    set(MICO_PLUGIN_LIBRARIES "")
    set(MICO_PLUGIN_COMPILE_DEFS "")

    # Get flow dep
    if(NOT ${BUNDLE_COMPILATION})
        find_package(flow REQUIRED)
        if(${flow_FOUND})
            set(MICO_PLUGIN_LIBRARIES ${MICO_PLUGIN_LIBRARIES} flow::flow)
            set(MICO_PLUGIN_COMPILE_DEFS ${MICO_PLUGIN_COMPILE_DEFS} HAS_FLOW)
        endif()
    else()
        set(flow_FOUND TRUE)
        set(MICO_PLUGIN_LIBRARIES ${MICO_PLUGIN_LIBRARIES} flow)
        set(MICO_PLUGIN_COMPILE_DEFS ${MICO_PLUGIN_COMPILE_DEFS} HAS_FLOW)
    endif()


    if(${BUNDLE_COMPILATION})
        set(MICO_PLUGIN_LIBRARIES ${MICO_PLUGIN_LIBRARIES} NodeEditor::nodes)
        set(MICO_PLUGIN_COMPILE_DEFS ${MICO_PLUGIN_COMPILE_DEFS} HAS_QTNODEEDITOR)
    else()
        message(FATAL_ERROR "NOT READY")
        find_package(NodeEditor REQUIRED)
        if(${NodeEditor_FOUND})
            # set(MICO_PLUGIN_INCLUDES "${MICO_PLUGIN_INCLUDES} ${NodeEditor_INCLUDE_DIRS})")
            # set(MICO_PLUGIN_LIBRARIES "${MICO_PLUGIN_LIBRARIES} ${NodeEditor_LIBRARIES})")
            # set(MICO_PLUGIN_COMPILE_DEFS "${MICO_PLUGIN_COMPILE_DEFS} HAS_QTNODEEDITOR")
        endif()
    endif()
  

    # Get othermico deps
    foreach(DEP ${IN_MICO_DEPS})
        if(NOT ${BUNDLE_COMPILATION})
            find_package(mico-${DEP} REQUIRED HINTS "/usr/local/lib/cmake/mico")
        endif()
        set(MICO_PLUGIN_LIBRARIES ${MICO_PLUGIN_LIBRARIES} mico-${DEP})
    endforeach()

    # Strip linking deps
    string(STRIP "${MICO_PLUGIN_LIBRARIES}" MICO_PLUGIN_LIBRARIES)
    
    # Create library
    add_library(${IN_PLUGIN_NAME} ${LIBRARY_MODE} ${IN_PLUGIN_SOURCES} ${IN_PLUGIN_HEADERS})
    target_compile_features(${IN_PLUGIN_NAME} PUBLIC cxx_std_17)
    target_include_directories(${IN_PLUGIN_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    if(WIN32)
        target_include_directories(${IN_PLUGIN_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    endif()

    add_library(${PROJECT_NAME}::${IN_PLUGIN_NAME} ALIAS ${IN_PLUGIN_NAME})

    target_include_directories(${IN_PLUGIN_NAME} PRIVATE ${MICO_PLUGIN_INCLUDES})
    target_link_libraries(${IN_PLUGIN_NAME} PRIVATE ${MICO_PLUGIN_LIBRARIES})
    target_compile_definitions(${IN_PLUGIN_NAME} PRIVATE ${MICO_PLUGIN_COMPILE_DEFS})

    # Plugin installation target
    if(WIN32)
        set(EXPORTED_PLUGIN_NAME ${IN_PLUGIN_NAME}_mplugin)
        add_library(${EXPORTED_PLUGIN_NAME} SHARED ${IN_PLUGIN_SOURCES})
        target_include_directories(${EXPORTED_PLUGIN_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${MICO_PLUGIN_INCLUDES})
        target_link_libraries(${EXPORTED_PLUGIN_NAME} PRIVATE ${MICO_PLUGIN_LIBRARIES})
        target_compile_definitions(${EXPORTED_PLUGIN_NAME} PRIVATE ${MICO_PLUGIN_COMPILE_DEFS})
    else(UNIX)
        set(EXPORTED_PLUGIN_NAME ${IN_PLUGIN_NAME})
    endif()

    #file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/plugins/resources/math)

    add_custom_target(flow_install_${IN_PLUGIN_NAME} ALL
                    COMMAND ${CMAKE_COMMAND} -E copy 	$<TARGET_FILE:${EXPORTED_PLUGIN_NAME}>
                    ${CMAKE_BINARY_DIR}/plugins
    )

    if(${IN_HAS_RESOURCES})
        add_custom_target(flow_install_${IN_PLUGIN_NAME}_resources ALL
                        COMMAND ${CMAKE_COMMAND} -E copy_directory 	${CMAKE_CURRENT_SOURCE_DIR}/resources 
                        ${CMAKE_BINARY_DIR}/plugins/resources
        )
        add_dependencies(flow_install_${IN_PLUGIN_NAME}_resources ${IN_PLUGIN_NAME})
    endif()

    add_dependencies(flow_install_${IN_PLUGIN_NAME} ${IN_PLUGIN_NAME})

endmacro(add_mplugin)

#----------------------------------------------------------------------------------------------------------------------
macro(mplugin_link_library)
    set(options IS_PUBLIC)
    set(oneValueArgs PLUGIN_NAME)
    set(multiValueArgs PLUGIN_LIBRARIES)
    cmake_parse_arguments(IN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    if(${IN_PLUGIN_NAME} STREQUAL "mico-visualizers")
        message(WARNING "I AM IN ${IN_PLUGIN_LIBRARIES}")
    endif()

    if(IS_PUBLIC)
        target_link_libraries(${IN_PLUGIN_NAME} PUBLIC ${IN_PLUGIN_LIBRARIES})

        if(WIN32)
            target_link_libraries(${IN_PLUGIN_NAME}_mplugin PUBLIC ${IN_PLUGIN_LIBRARIES})
        endif()
    else()
        target_link_libraries(${IN_PLUGIN_NAME} PRIVATE ${IN_PLUGIN_LIBRARIES})

        if(WIN32)
            target_link_libraries(${IN_PLUGIN_NAME}_mplugin PRIVATE ${IN_PLUGIN_LIBRARIES})
        endif()
    endif()

    
endmacro(mplugin_link_library)

#----------------------------------------------------------------------------------------------------------------------
macro(mplugin_include_directory)
    set(options IS_PUBLIC)
    set(oneValueArgs PLUGIN_NAME)
    set(multiValueArgs PLUGIN_INCLUDES)
    cmake_parse_arguments(IN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    if(IS_PUBLIC)
        target_include_directories(${IN_PLUGIN_NAME} PUBLIC ${PLUGIN_INCLUDES})
        
        if(WIN32)
            target_include_directories(${IN_PLUGIN_NAME}_mplugin PUBLIC ${PLUGIN_INCLUDES})
        endif()
    else()
        target_include_directories(${IN_PLUGIN_NAME} PRIVATE ${PLUGIN_INCLUDES})
            
        if(WIN32)
            target_include_directories(${IN_PLUGIN_NAME}_mplugin PRIVATE ${DEP})
        endif()
    endif()

endmacro(mplugin_include_directory)

#----------------------------------------------------------------------------------------------------------------------
macro(mplugin_compile_definition)
    set(options IS_PUBLIC)
    set(oneValueArgs PLUGIN_NAME)
    set(multiValueArgs PLUGIN_DEFINITIONS)
    cmake_parse_arguments(IN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    if(IS_PUBLIC)
        target_compile_definitions(${IN_PLUGIN_NAME} PUBLIC ${IN_PLUGIN_DEFINITIONS})
        
        if(WIN32)
            target_compile_definitions(${IN_PLUGIN_NAME}_mplugin PUBLIC ${IN_PLUGIN_DEFINITIONS})
        endif()
    else()
        target_compile_definitions(${IN_PLUGIN_NAME} PRIVATE ${IN_PLUGIN_DEFINITIONS})
        
        if(WIN32)
            target_compile_definitions(${IN_PLUGIN_NAME}_mplugin PRIVATE ${IN_PLUGIN_DEFINITIONS})
        endif()
    endif()
    
endmacro(mplugin_compile_definition)



#----------------------------------------------------------------------------------------------------------------------
macro(detect_OS)
    if(UNIX AND APPLE)	
        set(TARGET_OS mac)
    elseif(UNIX)
	find_program(LSB_RELEASE_EXEC lsb_release)
	execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
	    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
	    OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	set(TARGET_OS ${LSB_RELEASE_ID_SHORT})
    elseif(WIN32)
        set(TARGET_OS Windows)
    else()
        set(TARGET_OS Unknown)
    endif()
    
endmacro(detect_OS)
