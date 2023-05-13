################################################################################
###
### @file       cmake/force_include.cmake
###
### @project    EM7189
###
### @brief
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
SET(usegcc ${CMAKE_ARGV3})
SET(OUTPUT_FILE "${CMAKE_ARGV4}")
SET(LIBS_FILE  "${CMAKE_ARGV5}")
SET(SECTIONS_FILE "${CMAKE_ARGV6}")
SET(FORCE_INCLUDE "")

SET(ALL_SYMS )

FILE(READ ${LIBS_FILE} LIBS)
FOREACH(l ${LIBS})
    set(LIBRARIES ${LIBRARIES} ${l})
ENDFOREACH()

FILE(READ ${SECTIONS_FILE} SECTIONS)
SET(SECTION_REGEX "")


MACRO(FORCE_INCLUDE_NMAC NM_PATH SECTIONS LIBRARIES OUTPUT_FILE)
    ##
    ## Generate the regex when using the Metaware nmac tool.
    ##
    FOREACH(section ${SECTIONS})
        SET(SECTION_REGEX "${SECTION_REGEX}|DATA.*${section}.*")
    endFOREACH()
    STRING(SUBSTRING ${SECTION_REGEX} 1 -1 SECTION_REGEX)

    #message(STATUS "FORCE INCLUDE NMAC: ${OUTPUT_FILE}")
    #message(STATUS "LIBS: ${LIBRARIES}")
    #message(STATUS "SECTION_REGEX: ${SECTION_REGEX}")


    ### Grab NMAC output
    EXECUTE_PROCESS(COMMAND "${NM_PATH}" -q ${LIBRARIES} OUTPUT_VARIABLE nmac)

    STRING(REGEX REPLACE "\n" ";" nmac "${nmac}")
    FOREACH(line as ${nmac})
        STRING(REGEX MATCHALL "${SECTION_REGEX}" found ${line})
        IF(NOT "${found}" STREQUAL "")
            STRING(REGEX REPLACE "[ \t]+" ";" words "${found}")
            LIST(REVERSE words)
            LIST(GET words 0 symbol)

            #message("SYMBOL: ${symbol}")

            SET(FORCE_INCLUDE "${FORCE_INCLUDE}-u ${symbol}\n")
            LIST(APPEND ALL_SYMS "${symbol}")
        ENDIF(NOT "${found}" STREQUAL "")
    ENDFOREACH()

    FILE(WRITE ${OUTPUT_FILE} ${FORCE_INCLUDE})
ENDMACRO(FORCE_INCLUDE_NMAC)


MACRO(FORCE_INCLUDE_GNU_NM NM_PATH SECTIONS LIBRARIES OUTPUT_FILE)
    ##
    ## Generate the regex when using the arc-elf32-nm tool.
    ##
    FOREACH(section ${SECTIONS})
        SET(SECTION_REGEX "${SECTION_REGEX}|.*\\|.*\\|.*\\|.*\\|.*\\|.*\\|.*${section}$")
    ENDFOREACH()
    STRING(SUBSTRING ${SECTION_REGEX} 1 -1 SECTION_REGEX)

    #message(STATUS "LIBS: ${LIBRARIES}")
    #message(STATUS "SECTION_REGEX: ${SECTION_REGEX}")

    ### Grab NMAC output
    EXECUTE_PROCESS(COMMAND ${NM_PATH} -f sysv ${LIBRARIES} OUTPUT_VARIABLE nmac)

    STRING(REGEX REPLACE "\n" ";" nmac "${nmac}")
    FOREACH(line as ${nmac})
        #message("REGEX: ${SECTION_REGEX}")
        STRING(REGEX MATCHALL "${SECTION_REGEX}" found ${line})
        IF(NOT "${found}" STREQUAL "")
            #MESSAGE("FOUND: ${line}")
            STRING(REGEX REPLACE "\\|" ";" words "${found}")
            LIST(GET words 0 symbol)

            #message("SYMBOL: ${symbol}")

            SET(FORCE_INCLUDE "${FORCE_INCLUDE}-u ${symbol}\n")
            LIST(APPEND ALL_SYMS "${symbol}")

        ENDIF(NOT "${found}" STREQUAL "")
    ENDFOREACH()

    FILE(WRITE ${OUTPUT_FILE} ${FORCE_INCLUDE})
ENDMACRO(FORCE_INCLUDE_GNU_NM)
##
## Keep section requested.
##
IF(NOT "${SECTIONS}" STREQUAL "")
    IF(${usegcc})
        FIND_PROGRAM(NM_PATH arc-elf32-nm)
        FORCE_INCLUDE_GNU_NM(${NM_PATH} "${SECTIONS}" "${LIBRARIES}" "${OUTPUT_FILE}")
    ELSE()
        FIND_PROGRAM(NM_PATH nmac)
        FORCE_INCLUDE_NMAC(${NM_PATH} "${SECTIONS}" "${LIBRARIES}" "${OUTPUT_FILE}")
    ENDIF()
ENDIF(NOT "${SECTIONS}" STREQUAL "")


# Ensure that multiple libraries don't have the same required symbol.
IF(ALL_SYMS)
    SET(ALL_SYMS_NO_DUP ${ALL_SYMS})
    LIST(REMOVE_DUPLICATES ALL_SYMS_NO_DUP)

    IF(NOT ALL_SYMS STREQUAL ALL_SYMS_NO_DUP)
        FOREACH(item ${ALL_SYMS_NO_DUP})
            LIST(FIND ALL_SYMS ${item} position)
            LIST(REMOVE_AT ALL_SYMS ${position})
        ENDFOREACH()

        MESSAGE(FATAL_ERROR "Duplicate symbols found for ${ALL_SYMS}.")
    ENDIF()
ENDIF()
