################################################################################
###
### @file       cmake/allocations.cmake
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
SET(output  ${CMAKE_ARGV3})
SET(libfile ${CMAKE_ARGV4})

FILE(READ ${libfile} libs)

FOREACH(lib ${libs})

    EXECUTE_PROCESS(COMMAND sizeac -qut ${lib}
                    OUTPUT_VARIABLE sizes
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    STRING(REGEX REPLACE "TOTAL:" "" sizes "${sizes}")
    STRING(REGEX REPLACE "[ \t]" "" sizes "${sizes}")
    STRING(REGEX REPLACE "[+]" "," sizes "${sizes}")


    EXECUTE_PROCESS(COMMAND nmac -Ahq ${lib}
                    OUTPUT_VARIABLE symbols
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    STRING(REGEX MATCHALL "[0-9]+[ ]+[a-zA-Z]+[ ]+[a-zA-Z]+[ ]+.reclaim_data" reclaim_data "${symbols}")
    SET(RECLAIM_DATA_COUNT  0)
    FOREACH(line ${reclaim_data})
        STRING(REGEX REPLACE "[ \t]" ";" line "${line}")
        STRING(REGEX MATCH "[0-9]*" line ${line})
        MATH(EXPR RECLAIM_DATA_COUNT "${RECLAIM_DATA_COUNT} + ${line}")
    ENDFOREACH()

    STRING(REGEX MATCHALL "[0-9]+[ ]+[a-zA-Z]+[ ]+[a-zA-Z]+[ ]+.reclaim_text" reclaim_text "${symbols}")
    SET(RECLAIM_TEXT_COUNT  0)
    FOREACH(line ${reclaim_text})
        STRING(REGEX REPLACE "[ \t]" ";" line "${line}")
        STRING(REGEX MATCH "[0-9]*" line ${line})
        MATH(EXPR RECLAIM_TEXT_COUNT "${RECLAIM_TEXT_COUNT} + ${line}")
    ENDFOREACH()


    STRING(REGEX REPLACE "=" ",${RECLAIM_DATA_COUNT},${RECLAIM_TEXT_COUNT}," sizes "${sizes}")

    IF(NOT EXISTS ${output})
        FILE(APPEND ${output} "Library,text,data,bss,reclaim_data,reclaim_text,total\n")
    ENDIF()

    FILE(APPEND ${output} "${lib},${sizes}\n")
ENDFOREACH()