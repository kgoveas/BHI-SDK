################################################################################
###
### @file       arc.cmake
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
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

SET(ARC True)
SET(ARCH arc)
SET(TOOLCHAIN arc)
set(ARC_COMPILER )

# Overriding the compiler doesn't work under visual studio. Make sure we don't try to build the firmware.
STRING(FIND "${CMAKE_GENERATOR}" "Visual Studio" IS_VISUAL_STUDIO)
if("${IS_VISUAL_STUDIO}" EQUAL -1)
    FIND_PROGRAM(CCAC_PATH ccac)
    IF(CCAC_PATH AND NOT USE_GCC)
        SET(USE_GCC 0)
        SET(ARC_COMPILER ${CCAC_PATH})
        SET(ARC_AR ararc)
        SET(ARC_AR_ARGS qcr)
        FIND_PROGRAM(FLINT_PATH flint_ccac)
        IF(FLINT_PATH)
            SET(ARC_COMPILER ${FLINT_PATH})
            MESSAGE("flint_ccac found at ${FLINT_PATH}, enabling linting.")
        ENDIF(FLINT_PATH)
    ELSE()
        FIND_PROGRAM(GCC_PATH arc-elf32-gcc)
        IF(GCC_PATH)
            SET(USE_GCC 1)
            SET(ARC_COMPILER ${GCC_PATH})
            SET(TOOLCHAIN arc-gnu)
            SET(ARC_AR arc-elf32-ar)
            SET(ARC_AR_ARGS cr)
        ELSEIF(USE_GCC)
            MESSAGE(FATAL_ERROR "USE_GCC was specified, but a compatible arc-elf32-gcc was not found")
        ELSE()
            MESSAGE(WARNING "Unable to locate a usable ARC compiler (ccac or arc-elf32-gcc)")
            RETURN()
        ENDIF()
    ENDIF()

    SET(CMAKE_CXX_COMPILER_FORCED TRUE)
    SET(CMAKE_C_COMPILER_FORCED TRUE)

    SET(CMAKE_C_COMPILER   ${ARC_COMPILER})
    SET(CMAKE_CXX_COMPILER ${ARC_COMPILER})
    SET(CMAKE_ASM_COMPILER ${ARC_COMPILER})
    SET(CMAKE_C_AR         ${ARC_AR})
    SET(CMAKE_CXX_AR       ${ARC_AR})
    SET(CMAKE_ASM_AR       ${ARC_AR})

    # Ensure dependencies are generated proeprly... Looks like Compiler/GNU.cmake isn't being included properly. FIXME
    set(CMAKE_DEPFILE_FLAGS_C "-MD -MT <OBJECT> -MF \"<DEPFILE>\"")
    set(CMAKE_DEPFILE_FLAGS_CXX "-MD -MT <OBJECT> -MF \"<DEPFILE>\"")
    set(CMAKE_DEPFILE_FLAGS_ASM "-MD -MT <OBJECT> -MF \"<DEPFILE>\"")


    SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

    SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
    SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)

    SET(CMAKE_C_OUTPUT_EXTENSION ".o")

    SET(CMAKE_C_CREATE_STATIC_LIBRARY "${ARC_AR} ${ARC_AR_ARGS} <TARGET> <LINK_FLAGS> <OBJECTS>")
    SET(CMAKE_CXX_CREATE_STATIC_LIBRARY "${ARC_AR} ${ARC_AR_ARGS} <TARGET> <LINK_FLAGS> <OBJECTS>")
    SET(CMAKE_ASM_CREATE_STATIC_LIBRARY "${ARC_AR} ${ARC_AR_ARGS} <TARGET> <LINK_FLAGS> <OBJECTS>")
ENDIF()
