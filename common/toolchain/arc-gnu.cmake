################################################################################
###
### @file       common/toolchain/arc-gnu.cmake
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
SET(CMAKE_C_FLAGS "")
SET(CMAKE_ASM_FLAGS " -x assembler-with-cpp ")
set(CMAKE_EXE_LINKER_FLAGS " -e_start -Wl,--start-group -lm") # Link in the math library by default
SET(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE} -Xlinker -Map=<TARGET>.map -Wl,--end-group")
set(CMAKE_MODULE_LINKER_FLAGS "")
set(CMAKE_SHARED_LINKER_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS )

SET(ARC_TOOLCHAIN_LINKER_SCRIPT "-Wl,-T")

set(C_ASM_FLAGS
    -mno-sdata          # Don't assume any variables are in the small-data area.
    -Os                 # Optimize the code for size.
)

ADD_C_FLAGS( ${C_ASM_FLAGS} )
ADD_ASM_FLAGS( ${C_ASM_FLAGS} )


### Determine the version number of the compiler being used. This assumes that the last line from gcc -v ends with the version number 2018.09)
EXECUTE_PROCESS(COMMAND ${ARC_COMPILER} -v
                ERROR_VARIABLE stderr)
STRING(REGEX REPLACE "\n" ";" stderr "${stderr}")
LIST(GET stderr -2 stderr)

# eg. "2012.07)"
string(REGEX MATCH ".?([0-9]?[0-9]?[0-9]?[0-9]?)\\.([0-9]+)\\)" VERSION ${stderr})

SET(GCC_MAJOR ${CMAKE_MATCH_1})
SET(GCC_MINOR ${CMAKE_MATCH_2})

IF(GCC_MAJOR EQUAL 2018 AND GCC_MINOR EQUAL 09)
    # This version of GCC generates invalid assembly when accessing stack variables in some cases.
    # Enabling -fstack-protector-strong results in larger but correct code.
    MESSAGE("-- Building with the ${GCC_MAJOR}.${GCC_MINOR} compiler - enabling -fstack-protector-strong to work around compiler bug.")
    ADD_C_FLAGS(
        -fstack-protector-strong
    )
ENDIF()

