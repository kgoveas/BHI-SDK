################################################################################
###
### @file       utils/stuffelf/CMakeLists.txt
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
project(stuffelf)

if(MSVC)
    add_definitions(
        -FI \"${CMAKE_CURRENT_SOURCE_DIR}/functions.h\"
        -DROM_NAME=${ROM_NAME}
        -D__T7189__
        -DDEVICE=0x7189
        -DINTF_STREAMING
        -D_CRT_SECURE_NO_WARNINGS
        -D_CRT_NONSTDC_NO_DEPRECATE
        -wd4244
        -wd4018
        )
else()
    ADD_C_FLAGS(
        -Wall
        -Wno-char-subscripts
        -Wno-int-to-pointer-cast
        -Wno-pointer-to-int-cast
        -Wno-unknown-pragmas
        -Werror
        -D "'_lr(__x__)'=1"
        -D "'_sr(__x__, __y__)'="
        -D "'__aux(__x__)'="
        -DROM_NAME=${ROM_NAME}
        -D "'SleepCPU()'="
        ## Fix this. These should not be needed here.
        -D__T7189__
        -DDEVICE=0x7189
        -DINTF_STREAMING
        )
endif()


include_directories(
    ${EM718X_TOP}/boot_rom/includes
    ${EM718X_TOP}/common/includes
    ${EM718X_TOP}/libs/SensorInterface/includes
    ${EM718X_TOP}/libs/I2CInterface/includes
    ${EM718X_TOP}/libs/Config/includes
)

SET(SOURCES config.c crc32.c stuffelf.c)

if(MSVC)
    list(APPEND SOURCES getline.c)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 99)
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD_REQUIRED True)

target_link_libraries(${PROJECT_NAME} elfio)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/ EXPORT utils-targets)
install(EXPORT utils-targets DESTINATION cmake)
