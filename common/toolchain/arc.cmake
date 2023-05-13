################################################################################
###
### @file       common/toolchain/arc.cmake
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
SET(CMAKE_ASM_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-Bgrouplib")
SET(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE} -q -Wl,-m -Wl,-C -Wl,output=<TARGET>.map")
set(CMAKE_MODULE_LINKER_FLAGS "")
set(CMAKE_SHARED_LINKER_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS )

SET(ARC_TOOLCHAIN_LINKER_SCRIPT "-Wl,-A")

set(C_ASM_FLAGS
    -Hnosdata               # Don't assume any variables are in the small-data area.
    -Hnocopyr               # Disables the compiler version and copyright.
    -Os                     # Optimize the code for size.
    -mllvm                  # Additional arguments to forward to LLVM's option processing.
    -align-labels=false     # Disable alignment of labels.
    -Hinline_threshold=12   # Fix for 17.09: Adjust inlining behavior to match 16.12
)

ADD_C_FLAGS( ${C_ASM_FLAGS} )
ADD_ASM_FLAGS( ${C_ASM_FLAGS}
    -Hasopt=-offwarn=121
    -Hasopt=-on=assume_short
)

if(NOT ARC_COMPILER)
    RETURN()
endif()

### Determine the version number of the compiler being used. This assumes that the first line from ccac -q -v ends with the version number (M-2016.12)
EXECUTE_PROCESS(COMMAND ${ARC_COMPILER} -q -v
                OUTPUT_VARIABLE stdout
                ERROR_QUIET)


# eg. "M-2012.07"
string(REGEX MATCH ".?-([0-9]?[0-9]?[0-9]?[0-9]?)\\.([0-9]+)" VERSION ${stdout})

SET(METAWARE_MAJOR ${CMAKE_MATCH_1})
SET(METAWARE_MINOR ${CMAKE_MATCH_2})

IF(METAWARE_MAJOR LESS 2018 AND (METAWARE_MAJOR EQUAL 2017 AND METAWARE_MINOR LESS 12))
    # Compiler versions from 2017.12 and beyond now use r30 as scratch.
    MESSAGE(FATAL_ERROR "MetaWare compilers before 2017.12 are not supported. Please upgrade to a newer compiler.")
ENDIF()
