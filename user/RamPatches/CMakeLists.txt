################################################################################
###
### @file       user/RamPatches/CMakeLists.txt
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
SET(PROJECT_NAME ${boad_ram_patch_library})

PROJECT(${PROJECT_NAME} C)

include_directories(
    ../../common/includes
    ../../libs/Hooks/includes
    ../../libs/HostInterface/includes
    ../../libs/HostInterfaceStreaming
    ../../libs/SensorInterface/includes
    ../../libs/Time/includes
    ../../libs/I2CInterface/includes
    ../../libs/Security/includes
    ../../libs/Debug/includes
    ../../libs/Outerloop/includes
    ../../libs/Config/includes
    ../../libs/Ram/includes
    ../../libs/Bootloader/includes
    ../../libs/openrtos/include
    ../../libs/openrtos/portable/Metaware/EMx_MPU
)

if(NOT TARGET user-ram-patches)
    SET(HEADERS )
    SET(SOURCES )
    foreach(hd ${RAM_PATCHES_HEADERS})
        set(hdr ${hd}.h)
        list(APPEND HEADERS ${hdr})
    endforeach()

    # Build an object library of the default ram-patches so that we don't have to compile them again.
    foreach(p ${RAM_PATCHES})
        set(src ${p}.c)
        list(APPEND SOURCES ${src})
    endforeach()

    add_library(user-ram-patches OBJECT ${SOURCES})
    target_compile_options(user-ram-patches PRIVATE  ${USER_RAM_C_FLAGS})

    IF(BUILD_FLASH)
        add_library(user-ram-patches-flash OBJECT ${SOURCES})
        target_compile_options(user-ram-patches-flash PRIVATE  ${USER_FLASH_C_FLAGS})
    ENDIF()

    IF(NOT SDK)
        install(FILES ${SOURCES} DESTINATION user/RamPatches)
        install(FILES ${HEADERS} DESTINATION user/RamPatches)
        install(FILES CMakeLists.txt DESTINATION user/RamPatches)
    ENDIF()
endif()

SET(ADDITIONAL_SOURCES )
foreach(p ${board_specific_ram_patches})
   # Find all the additional ram patches to add for this board.
   set(src ${p}.c)
   list(APPEND ADDITIONAL_SOURCES ${src})
endforeach()

IF(NOT SDK)
    install(FILES ${ADDITIONAL_SOURCES} DESTINATION user/RamPatches)
ENDIF()

add_library(${PROJECT_NAME} $<TARGET_OBJECTS:user-ram-patches> ${ADDITIONAL_SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE  ${USER_RAM_C_FLAGS})

IF(BUILD_FLASH)
    add_library(${PROJECT_NAME}-flash $<TARGET_OBJECTS:user-ram-patches-flash> ${ADDITIONAL_SOURCES})
    target_compile_options(${PROJECT_NAME}-flash PRIVATE  ${USER_FLASH_C_FLAGS})
ENDIF()

IF(NOT SDK)
    install(FILES custom_bsx_low_power_sensors.c DESTINATION user/RamPatches)
    install(FILES RamPatches_ReadMe.txt DESTINATION user/RamPatches)
ENDIF()
