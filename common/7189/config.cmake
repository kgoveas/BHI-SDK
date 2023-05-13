################################################################################
###
### @file       common/7189/config.cmake
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
SET(HW_ROM0_START 0x100000)
SET(HW_ROM0_SIZE 65536)
SET(HW_ROM1_START 0x110000)
SET(HW_ROM1_SIZE 65536)
SET(HW_ROM2_START 0x120000)
SET(HW_ROM2_SIZE 16384)

SET(HW_ROM_START 0x100000)
SET(HW_ROM_SIZE 147456)

SET(HW_OTP_START 0x0)
SET(HW_OTP_SIZE 128)

SET(7189_C_ASM_FLAGS
    -D__T7189__
    -DDEVICE=0x7189
)

IF(${TOOLCHAIN} STREQUAL arc)
    SET(7189_C_ASM_FLAGS ${7189_C_ASM_FLAGS}
            @"${EM718X_TOP}/common/${TARGET_HW}/ccac.arg"
        )
ELSE()
    SET(7189_C_ASM_FLAGS ${7189_C_ASM_FLAGS}
            @"${EM718X_TOP}/common/${TARGET_HW}/gcc.arg"
        )
ENDIF()

ADD_C_FLAGS(
    ${7189_C_ASM_FLAGS}
)

ADD_ASM_FLAGS(
    ${7189_C_ASM_FLAGS}
)

ADD_CXX_FLAGS(
    ${7189_C_ASM_FLAGS}
)

IF(BOOT_ROM_SOURCE)
    SET(ROM_TYPE_INSTALL_PATH ${CMAKE_CURRENT_LIST_DIR}/${ROM_TYPE})

    install(FILES ${CMAKE_CURRENT_LIST_DIR}/peripherals.sym
            DESTINATION common/${TARGET_HW}/)
ENDIF()

IF(NOT SDK)
    IF(BUILD_KERNEL)
        install(DIRECTORY
            ${CMAKE_CURRENT_LIST_DIR}/${RAM_TYPE}
            ${CMAKE_CURRENT_LIST_DIR}/${FLASH_TYPE}
            ${CMAKE_CURRENT_LIST_DIR}/${FLASH_BL_TYPE}
            DESTINATION common/${TARGET_HW})
    ENDIF()

    install(FILES
        ${CMAKE_CURRENT_LIST_DIR}/gcc.arg
        ${CMAKE_CURRENT_LIST_DIR}/ccac.arg
        ${CMAKE_CURRENT_LIST_DIR}/config.cmake
        ${CMAKE_CURRENT_LIST_DIR}/link_map.cmd
        DESTINATION common/${TARGET_HW}
    )

    install(DIRECTORY
        ${ROM_TYPE_INSTALL_PATH}
        ${CMAKE_CURRENT_LIST_DIR}/${USER_RAM_TYPE}
        ${CMAKE_CURRENT_LIST_DIR}/includes
        DESTINATION common/${TARGET_HW}
    )

    IF(GENERATES_SDK OR BUILD_FLASH)
        install(FILES
            ${CMAKE_CURRENT_LIST_DIR}/link_map_ssbl.cmd
            DESTINATION common/${TARGET_HW}
        )
        install(DIRECTORY
            ${CMAKE_CURRENT_LIST_DIR}/${USER_FLASH_TYPE}
            DESTINATION common/${TARGET_HW}
        )
    ENDIF()
ENDIF()

IF(USE_GCC)
    SET(COMPONENT_PREFIX GCC)
    SET(DEST_PREFIX gcc)
ENDIF()

if(SDK)
    SET(INSTALL_COMPONENTS ${COMPONENT_PREFIX}FW ${COMPONENT_PREFIX}ELF)
ELSE()
    SET(INSTALL_COMPONENTS SDK ${COMPONENT_PREFIX}FW ${COMPONENT_PREFIX}ELF)
ENDIF()

SET(INSTALL_ICRC false)
SET(INSTALL_ISIGN_REST false)
SET(INSTALL_IENCRYPT false)

IF(EXISTS ${EM718X_TOP}/.internal)
    LIST(APPEND INSTALL_UTILS isign keygen)
    LIST(APPEND INSTALL_COMPONENTS isign keygen)
ENDIF()

IF(GENERATES_SDK OR BUILD_ROM)
    SET(INSTALL_ISIGN_REST true)
    SET(INSTALL_ICRC true)
ENDIF()

IF(NOT SDK AND SUPPORT_ENCRYPTED_FW)
    SET(INSTALL_IENCRYPT true)
ENDIF()

# How to sign the kernel image
IF((${SECURITY_TYPE} STREQUAL SIGNED_UNSIGNED) OR (${SECURITY_TYPE} STREQUAL SIGNED_ALL))
    SET(KERNEL_SUFFIX "fws")
    IF(${SIGN_METHOD} STREQUAL SIGN_LOCAL)
        SET(KERNEL_SIGNER isign -s -k "${EM718X_TOP}/utils/isign/key.pri")
        IF(BUILD_KERNEL AND NOT EXISTS "${EM718X_TOP}/utils/isign/key.pri")
            MESSAGE(FATAL_ERROR "Unable to locate kernel private key at ${EM718X_TOP}/utils/isign/key.pri")
        ENDIF()
    ELSE()
        IF(NOT DEFINED KERNEL_EXT_SIGNER)
            SET(KERNEL_EXT_SIGNER "'${EM718X_TOP}/utils/isign_rest/rest_signing_kernel.sh'")
        ELSE()
            SET(KERNEL_EXT_SIGNER "'${KERNEL_EXT_SIGNER}' '${EXT_SIGNER_DIR}'")
        ENDIF()
        SET(KERNEL_SIGNER isign_rest -s --extsign "${KERNEL_EXT_SIGNER}")
    ENDIF()
ELSEIF(${SECURITY_TYPE} STREQUAL UNSIGNED_ALL)
    SET(KERNEL_SUFFIX "fwu")
    SET(KERNEL_SIGNER ${UTILS_BIN_DIR}/icrc)
ENDIF()

# How to sign the user image
IF((${SECURITY_TYPE} STREQUAL SIGNED_UNSIGNED) OR (${SECURITY_TYPE} STREQUAL UNSIGNED_ALL))
    SET(USER_SUFFIX "fwu")
    SET(USER_SIGNER ${UTILS_BIN_DIR}/icrc)
    SET(INSTALL_ICRC true)
ELSEIF(${SECURITY_TYPE} STREQUAL SIGNED_ALL)
    SET(USER_SUFFIX "fws")
    IF(${SIGN_METHOD} STREQUAL SIGN_LOCAL)
        SET(USER_PRIVATE_KEY "${EM718X_TOP}/utils/isign/user_ram_default_key.pri")
        SET(USER_SIGNER isign -s -k "${USER_PRIVATE_KEY}")
        IF(NOT EXISTS "${USER_PRIVATE_KEY}")
            MESSAGE(FATAL_ERROR "Unable to locate user private key at ${USER_PRIVATE_KEY}")
        ENDIF()
    ELSE(${SIGN_METHOD} STREQUAL SIGN_SERVER)
        IF(USER_EXT_SIGNER STREQUAL "")
            SET(USER_EXT_SIGNER   "'${EM718X_TOP}/utils/isign_rest/rest_signing_user.sh'")
        ELSE()
            SET(USER_EXT_SIGNER "'${USER_EXT_SIGNER}' '${EXT_SIGNER_DIR}'")
        ENDIF()
        SET(USER_SIGNER isign_rest -s --extsign "${USER_EXT_SIGNER}")
        SET(INSTALL_ISIGN_REST true)
    ENDIF()
ELSE()
    MESSAGE(FATAL_ERROR "SECURITY_TYPE must be either SIGNED_UNSIGNED, SIGNED_ALL, or UNSIGNED_ALL")
ENDIF()

IF(ENCRYPT_USER_PAYLOAD AND NOT SUPPORT_ENCRYPTED_FW)
    MESSAGE(FATAL_ERROR "SDK does not support encrypted firmware. Unable to encrypt the user payload")
ELSEIF(ENCRYPT_USER_PAYLOAD OR SUPPORT_ENCRYPTED_FW)
    SET(AES_KEY ${CMAKE_SOURCE_DIR}/keys/custkey.aes)
    IF(ENCRYPT_USER_PAYLOAD AND NOT EXISTS ${AES_KEY})
        MESSAGE(FATAL_ERROR "Unable to locate AES key at ${AES_KEY}")
    ENDIF()
    IF(NOT SDK AND EXISTS ${AES_KEY})
        MESSAGE("-- User AES key: ${AES_KEY}")
        # Archive the AES key used for signinig
        install(FILES ${AES_KEY} COMPONENT custkey-aes DESTINATION keys RENAME custkey.aes)
        LIST(APPEND INSTALL_COMPONENTS custkey-aes)
    ENDIF()
ENDIF()

IF(INSTALL_ISIGN_REST)
    LIST(APPEND INSTALL_UTILS isign_rest)
ENDIF()
IF(INSTALL_ICRC)
    LIST(APPEND INSTALL_UTILS icrc)
ENDIF()
IF(INSTALL_IENCRYPT)
    LIST(APPEND INSTALL_UTILS iencrypt)
ENDIF()


IF(BOOT_ROM_SOURCE)
    if(NOT ROM_TYPE)
        message(FATAL_ERROR "ROM_TYPE not defined")
    endif()

    message(STATUS "ROM Type: ${ROM_TYPE}")
    include(${CMAKE_CURRENT_LIST_DIR}/${ROM_TYPE}/config.cmake)
ELSE()
    # Allow FPGAs to be built if fpga support exists.
    IF(EXISTS ${EM718X_TOP}/utils/fpga/fpga.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/${ROM_TYPE}/config.cmake)
    ENDIF()
ENDIF()

if(NOT RAM_TYPE)
    message(FATAL_ERROR "RAM_TYPE not defined")
endif()

if(NOT FLASH_TYPE)
    message(FATAL_ERROR "FLASH_TYPE not defined")
endif()

if(NOT USER_RAM_TYPE)
    message(FATAL_ERROR "USER_RAM_TYPE not defined")
endif()

if(NOT USER_FLASH_TYPE)
    message(FATAL_ERROR "USER_FLASH_TYPE not defined")
endif()

message(STATUS "RAM Type: ${RAM_TYPE}")
include(${CMAKE_CURRENT_LIST_DIR}/${RAM_TYPE}/config.cmake OPTIONAL)

message(STATUS "User RAM Type: ${USER_RAM_TYPE}")
include(${CMAKE_CURRENT_LIST_DIR}/${USER_RAM_TYPE}/config.cmake)

IF(BUILD_FLASH)
    message(STATUS "Flash Type: ${FLASH_TYPE}")
    include(${CMAKE_CURRENT_LIST_DIR}/${FLASH_TYPE}/config.cmake OPTIONAL)

    message(STATUS "User Flash Type: ${USER_FLASH_TYPE}")
    include(${CMAKE_CURRENT_LIST_DIR}/${USER_FLASH_TYPE}/config.cmake)

    message(STATUS "Flash Bootloader Type: ${FLASH_BL_TYPE}")
    include(${CMAKE_CURRENT_LIST_DIR}/${FLASH_BL_TYPE}/config.cmake OPTIONAL)
ENDIF()
