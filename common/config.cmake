################################################################################
###
### @file       common/config.cmake
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
if("${DIST_TYPE}" STREQUAL "")
    message(FATAL_ERROR "DIST_TYPE must be set")
endif()

SET(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR})

SET(JLI_SECTION jlitab)
SET(BOOT_RAM_SECTION dataram)

set(JLI "${CMAKE_BINARY_DIR}/jli.s")

include(${CMAKE_CURRENT_LIST_DIR}/macros.cmake)

if(TOOLCHAIN)
    include(${CMAKE_CURRENT_LIST_DIR}/toolchain/${TOOLCHAIN}.cmake)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/config.${DIST_TYPE}.cmake)

if(NOT ARC_COMPILER)
    set(GENERATES_SDK 0)
endif()

if(TARGET_HW AND ARC_COMPILER)
    include(${CMAKE_CURRENT_LIST_DIR}/${TARGET_HW}/config.cmake)
endif()

set(ROM_IMAGE ${ROM_NAME}.elf)
set(ROM_DIR  "${CMAKE_BINARY_DIR}/${ROM_NAME}")
set(ROM_PATH "${CMAKE_BINARY_DIR}/${ROM_NAME}/${ROM_IMAGE}")

SET(TARGET_HW_COMMON ${CMAKE_CURRENT_LIST_DIR}/${TARGET_HW})


SET(C_ASM_FLAGS
    -DVERSION=${SVN_REVISION}
    -DVERSION_BUGFIX=${SVN_REVISION}
    -DVERSION_MAJOR=${RELEASE_MAJOR}
    -DVERSION_MINOR=${RELEASE_MINOR}
    -DBUILD_DATE=""
    -DBUILD_TIME=""

    #FIXME: these are device specifc and should *NOT* be here. Move to the config.7189.cmake file.
    -DI2C_QUEUE_SIZE=40
    -DCONFIG_HAS_FLOAT_MATRIX=0
    -DCONFIG_HAS_BOSCH_EXT_DATA=1
    -DLARGE_DECIMATION
    -DPLACEHOLDER_SENSOR_API
    -DVIRTUAL_MODE_CHANGE
    -UVIRTUAL_CURRENT_CONSUMPTION # VIRTUAL_CURRENT_CONSUMPTION disabled.
    -DVIRTUAL_EXPANSION_DATA
    -DHAS_ACTUAL_RATE
)

ADD_C_FLAGS(
    ${C_ASM_FLAGS}
)

ADD_ASM_FLAGS(
    ${C_ASM_FLAGS}
)

ADD_CXX_FLAGS(
    ${C_ASM_FLAGS}
)

IF(${ARC})
ADD_LD_FLAGS(
)
ENDIF(${ARC})



set(HOST_INTERFACE_LIB HostInterface${HOST_INTERFACE})

IF(${HOST_INTERFACE} MATCHES "Streaming")
    ADD_C_FLAGS(-DINTF_STREAMING)
ELSEIF(${HOST_INTERFACE} MATCHES "AndroidL")
    ADD_C_FLAGS(-DINTF_ANDROIDL)
ELSEIF(${HOST_INTERFACE} MATCHES "Android")
    ADD_C_FLAGS(-DINTF_ANDROID)
ELSEIF(${HOST_INTERFACE} MATCHES "Legacy")
    ADD_C_FLAGS(-DINTF_LEGACY)
ELSE()
    ADD_C_FLAGS(-DINTF_FRAMEWORK)
ENDIF()

if(NOT "${BOARDS}" STREQUAL "")
    SET_DRIVERS(${BOARDS})
endif()

IF(GENERATES_SDK)
    install(FILES
        ${CMAKE_CURRENT_LIST_DIR}/cpack.cmake
        DESTINATION common)

    IF(NOT NEXT_SDK)
        message(FATAL "The current DIST_TYPE (${DIST_TYPE}) states that it generates an SDK, however the NEXT_SDK variable in undefined.")
    ENDIF()

    install(FILES
            ${CMAKE_CURRENT_LIST_DIR}/config.${NEXT_SDK}.cmake
            DESTINATION common)
ENDIF()

IF(NOT SDK)
    install(FILES
            ${CMAKE_CURRENT_LIST_DIR}/cpack.cmake
            ${CMAKE_CURRENT_LIST_DIR}/macros.cmake
            ${CMAKE_CURRENT_LIST_DIR}/config.cmake
            ${CMAKE_CURRENT_LIST_DIR}/config.${DIST_TYPE}.cmake
            ${CMAKE_CURRENT_LIST_DIR}/config.host.cmake
            DESTINATION common)

    install(DIRECTORY
            ${CMAKE_CURRENT_LIST_DIR}/toolchain
            ${CMAKE_CURRENT_LIST_DIR}/includes
            DESTINATION common)
ENDIF()
