################################################################################
###
### @file       utils/backtrace/CMakeLists.txt
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
cmake_minimum_required(VERSION 3.5.0)

find_path (LIBELF_INCLUDE_DIRS
    NAMES
        gelf.h
    PATHS
        /usr/include
        /usr/include/libelf
        /usr/local/include
        /usr/local/include/libelf
        /opt/local/include
        /opt/local/include/libelf
        /sw/include
        /sw/include/libelf
        ENV CPATH)

if(NOT LIBELF_INCLUDE_DIRS)
    message(STATUS "libelf-dev needed for backtrace utility. Backtrace utility will not be compiled.")
    return()
endif()

project(backtrace)

include(ExternalProject)
ExternalProject_add(libdwarf
    SOURCE_DIR        "${CMAKE_CURRENT_SOURCE_DIR}/libdwarf"
    BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/libdwarf"
    CONFIGURE_COMMAND "${CMAKE_CURRENT_BINARY_DIR}/libdwarf/configure"
    BUILD_COMMAND     "make"
)

ExternalProject_add_step(libdwarf permission
    DEPENDERS configure
    COMMAND find . -name "configure" | xargs chmod +x
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

ExternalProject_add_step(libdwarf copy
    COMMAND ${CMAKE_COMMAND} -E copy_directory
              "${CMAKE_CURRENT_SOURCE_DIR}/libdwarf"
              "${CMAKE_CURRENT_BINARY_DIR}/libdwarf"
    DEPENDERS configure permission
)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

include_directories(
    "${CMAKE_CURRENT_BINARY_DIR}/libdwarf"
)

link_directories("${CMAKE_CURRENT_BINARY_DIR}/libdwarf")

add_executable(backtrace backtrace.c crc32.c)
add_dependencies(backtrace libdwarf)
target_link_libraries(backtrace dwarf elf)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/ EXPORT utils-targets)
install(EXPORT utils-targets DESTINATION cmake)
