################################################################################
###
### @file       cmake/clang-format.cmake
###
### @project    util-template
###
### @brief      CMake script for calling clang-format.
###
### @classification  Confidential
###
################################################################################
###
################################################################################
###
### @copyright Copyright (C) 2019 EM Microelectronic
### @cond
###
### All rights reserved.
###
### Redistribution and use in source and binary forms, with or without
### modification, are permitted provided that the following conditions are met:
### 1. Redistributions of source code must retain the above copyright notice,
### this list of conditions and the following disclaimer.
### 2. Redistributions in binary form must reproduce the above copyright notice,
### this list of conditions and the following disclaimer in the documentation
### and/or other materials provided with the distribution.
###
################################################################################
###
### THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
### AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
### IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
### ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
### LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
### CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
### SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
### INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
### CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
### ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
### POSSIBILITY OF SUCH DAMAGE.
### @endcond
################################################################################

if(CMAKE_SCRIPT_MODE_FILE)
    IF(NOT FORMAT_SRCS)
        MESSAGE("No source files to check")
        RETURN()
    ENDIF()

    FIND_PROGRAM(CLANG_FORMAT clang-format)
    IF(NOT CLANG_FORMAT)
        MESSAGE(WARNING "Clang-format not found.")
        RETURN()
    ENDIF()

    #if clang-format version is not em_custom return
    EXECUTE_PROCESS(COMMAND ${CLANG_FORMAT} --version OUTPUT_VARIABLE CLANG_OUTPUT)
    STRING(FIND "${CLANG_OUTPUT}" "em_custom" FIND_RESULT)
    IF(FIND_RESULT LESS 0)
        MESSAGE(FATAL_ERROR "EM-specific clang-format version not found. Skipping clang-format.")
        RETURN()
    ENDIF()

    IF(${FORMAT})
        STRING(REPLACE ";" "\n\t" files "${FORMAT_SRCS}")
        MESSAGE("Formatting files:\n\t${files}")
        EXECUTE_PROCESS(COMMAND ${CLANG_FORMAT} -style=file -i ${FORMAT_SRCS})
    ELSE()
        SET(VALID True)
        FOREACH(file ${FORMAT_SRCS})
            EXECUTE_PROCESS(
                COMMAND ${CLANG_FORMAT} -style=file -output-replacements-xml ${file}
                OUTPUT_VARIABLE FORMAT_OUTPUT
                RESULTS_VARIABLE RESULT)
            IF(NOT RESULT STREQUAL "0")
                MESSAGE(FATAL_ERORR "Unable to run ${CLANG_FORMAT} (returned ${RESULT})")
            ENDIF()
            STRING(REGEX MATCH "<replacement " STATUS "${FORMAT_OUTPUT}")
            IF(${STATUS} MATCHES "<replacement ")
                MESSAGE(STATUS "Invalid formatting for file: ${file}")
                SET(VALID False)
            ENDIF()
        ENDFOREACH()

        IF(VALID)
            MESSAGE("Formating Valid.")
        ELSE()
            MESSAGE(FATAL_ERROR "Formating Required run: ${CMAKE_COMMAND} --build . --target clang-format")
        ENDIF()
    ENDIF()

    RETURN()
ELSE()
    # Take a target and extract its source files so they can be added to
    # clang-format. This will then be used in the formatting test, the clang-format
    # reformat tool, and the doxygen documentation generation.

    FUNCTION(format_target_sources TARGET_NAME)
        SET(REL_SOURCES )
        GET_TARGET_PROPERTY(LOCAL_SOURCES ${TARGET_NAME} SOURCES)
        FOREACH(source ${LOCAL_SOURCES})
            get_filename_component(path ${source} ABSOLUTE)
            file(RELATIVE_PATH rel_path ${CMAKE_BINARY_DIR} ${path})
            LIST(APPEND REL_SOURCES ${rel_path})
        ENDFOREACH()
        SET_PROPERTY(TARGET clang-format APPEND PROPERTY FORMAT_SRCS "${REL_SOURCES}")
    ENDFUNCTION()

    #Configure Clang Format tools and tests
    ADD_CUSTOM_TARGET(clang-format
        COMMAND ${CMAKE_COMMAND} "-DFORMAT_SRCS=$<TARGET_PROPERTY:clang-format,FORMAT_SRCS>" -DFORMAT=1 -P ${CMAKE_CURRENT_LIST_FILE} VERBATIM
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

    ADD_CUSTOM_TARGET(check-format
        COMMAND ${CMAKE_COMMAND} "-DFORMAT_SRCS=$<TARGET_PROPERTY:clang-format,FORMAT_SRCS>" -DFORMAT=0 -P ${CMAKE_CURRENT_LIST_FILE} VERBATIM
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

    FIND_PROGRAM(CLANG_FORMAT clang-format)
    IF(NOT CLANG_FORMAT)
        MESSAGE(WARNING "Clang-format not found.")
        RETURN()
    ENDIF()

    EXECUTE_PROCESS(COMMAND ${CLANG_FORMAT} --version OUTPUT_VARIABLE CLANG_OUTPUT)
    STRING(FIND "${CLANG_OUTPUT}" "em_custom" FIND_RESULT)
    IF(FIND_RESULT LESS 0)
        MESSAGE(STATUS "EM-specific clang-format version not found. Skipping clang-format.")
        RETURN()
    ELSE()
        ADD_PRE_TEST(check-format)
    ENDIF()
ENDIF()
