################################################################################
###
### @file       utils/CMakeLists.txt
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
cmake_minimum_required(VERSION 3.5.1)

project(utils)

set(DIST_TYPE host CACHE STRING "" FORCE)

SET(IN_UTILS TRUE)

include(../config.cmake)
include(../cmake/clang-format.cmake)

set(EXCLUDE_UTILS )

if(MSVC)
    # Statically link in needed runtime dependencies.
    set(CMAKE_CXX_FLAGS_RELEASE "/MT")
    set(CMAKE_C_FLAGS_RELEASE "/MT")
    set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
    set(CMAKE_C_FLAGS_DEBUG "/MTd")

    ADD_C_FLAGS(
        # /showIncludes
        /D_CRT_SECURE_NO_WARNINGS
        /Wall
        /WX
        /Wv:18
        /wd4464 # relative include path with ../
        /wd4214 # Nonstandard bitfield type other than int.

        /wd4710 # Function not inlined
        /wd4711 # Function was inlined
        /wd4668 # Not defined as macro
        /wd4820 # Padding added
    )

    list(APPEND EXCLUDE_UTILS backtrace)
else()
    ADD_C_FLAGS(
        -Wall
        -Werror
    )
endif()

if(NOT "genotp" IN_LIST UTILS)
    list(APPEND UTILS "genotp")
endif()

if(NOT "unifdef" IN_LIST UTILS)
    list(APPEND UTILS "unifdef")
endif()

if(NOT "isign_rest" IN_LIST UTILS)
    list(APPEND UTILS "isign_rest")
endif()

if(NOT "icrc" IN_LIST UTILS)
    list(APPEND UTILS "icrc")
endif()

if(NOT "isign" IN_LIST UTILS)
    list(APPEND UTILS "isign")
endif()

if(NOT "keygen" IN_LIST UTILS)
    list(APPEND UTILS "keygen")
endif()

if(NOT "iencrypt" IN_LIST UTILS)
    list(APPEND UTILS "iencrypt")
endif()

add_subdirectory(elfio)
add_subdirectory(libs)

foreach(util ${UTILS})
    if( NOT ${util} IN_LIST EXCLUDE_UTILS )
        if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/${util}/CMakeLists.txt)
            message("Building utility ${util}")
            add_subdirectory(${util})
        else()
            message("Skipping utility ${util}")
        endif()
    else()
        message("Excluding utility ${util}")
    endif()
endforeach(util ${UTILS})
