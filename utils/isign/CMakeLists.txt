################################################################################
###
### @file       utils/isign/CMakeLists.txt
###
### @project    EM7189
###
### @brief      isign cmake file
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
cmake_minimum_required(VERSION 3.5.0)
project(isign)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/includes"
)

add_library(rdRand          STATIC IMPORTED)
add_library(SHA256          STATIC IMPORTED)
add_library(ECC_FW_Generate STATIC IMPORTED)
add_library(ECC_FW_Verify   STATIC IMPORTED)

if(MSVC) # Windows Take care of a few warnings

set_target_properties(rdRand          PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib/rdRand.lib")
set_target_properties(SHA256          PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib/SHA256.lib")
set_target_properties(ECC_FW_Generate PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib/ECC_FW_Generate.lib")
set_target_properties(ECC_FW_Verify   PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib/ECC_FW_Verify.lib")

add_definitions(
    -D_CRT_SECURE_NO_DEPRECATE
    -Dpopen=_popen
    -Dpclose=_pclose
)

set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:MSVCRT /ignore:4099" )
string(REPLACE "/INCREMENTAL " "/INCREMENTAL:NO " CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
string(REPLACE "/INCREMENTAL " "/INCREMENTAL:NO " CMAKE_MODULE_LINKER_FLAGS_DEUBG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG}")
string(REPLACE "/INCREMENTAL " "/INCREMENTAL:NO " CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}")

set(CMAKE_MODULE_LINKER_FLAGS_DEBUG  "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} /INCREMENTAL:NO /OPT:NOREF /OPT:NOICF")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG     "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /INCREMENTAL:NO /OPT:NOREF /OPT:NOICF")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG  "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /INCREMENTAL:NO /OPT:NOREF /OPT:NOICF")

else(MSVC) # Linux build

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG(-no-pie HAS_NO_PIE)
IF(HAS_NO_PIE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")
ENDIF()

set_target_properties(rdRand          PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib-Linux/librdRand.a")
set_target_properties(SHA256          PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib-Linux/libSHA256.a")
set_target_properties(ECC_FW_Generate PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib-Linux/libECC_FW_Generate.a")
set_target_properties(ECC_FW_Verify   PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/seclib-Linux/libECC_FW_Verify.a")

endif(MSVC)

# Source files
add_executable(isign isign.cpp crc32.cpp)
# Add in libraries
target_link_libraries(isign ECC_FW_Generate ECC_FW_Verify SHA256 rdRand)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/ EXPORT utils-targets)
install(EXPORT utils-targets DESTINATION cmake)
