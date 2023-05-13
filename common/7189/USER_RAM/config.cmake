################################################################################
###
### @file       common/7189/USER_RAM/config.cmake
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
SET(USER_RAM_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/vectors.c
    #${CMAKE_CURRENT_LIST_DIR}/extensions.s
    ${CMAKE_CURRENT_LIST_DIR}/EM718xConfig.c
    ${CMAKE_CURRENT_LIST_DIR}/fwHeader.c
)

SET(USER_RAM_C_FLAGS
    ${PURGE}
    -DNO_JLI_CALLS
)

SET(USER_RAM_INITDAT
    !data
    !lit
    !bss
    ~.ram_keys
    ~.fw_header
    ~.fw_header_offset
    ~.em7189_descriptor
    .phys_sensor_descriptors
    .virt_sensor_descriptors
    .timer_sensor_descriptors
    ~.reclaim
    ~.vectors
)

SET(USER_RAM_LD_FLAGS
    ${USER_RAM_LD_FLAGS}
    -u gFwHeader
    -u gUserEntries
    -u header_copy_offset
    -Bdefine:_BUILD_YEAR=${BUILD_YEAR}
    -Bdefine:_BUILD_MONTH=${BUILD_MONTH}
    -Bdefine:_BUILD_DAY=${BUILD_DAY}
    -Bdefine:_BUILD_HOUR=${BUILD_HOUR}
    -Bdefine:_BUILD_MINUTE=${BUILD_MINUTE}
    -Bdefine:_BUILD_SECOND=${BUILD_SECOND}
    -Bdefine:USER_VERSION=${REVISION}
    ${PURGE}
)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/config.${ROM_TAG}.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/config.${ROM_TAG}.cmake)
endif()

IF(ENCRYPT_USER_PAYLOAD)
    SET(USER_KERNEL_IMAGE kernel-aes)
ELSE()
    SET(USER_KERNEL_IMAGE kernel)
ENDIF()

SET(USER_RAM_DEPENDS lib${USER_KERNEL_IMAGE} utils)

STRING(SUBSTRING ${BUILD_HASH} 0 2 HASH_B0)
STRING(SUBSTRING ${BUILD_HASH} 2 2 HASH_B1)
STRING(SUBSTRING ${BUILD_HASH} 4 2 HASH_B2)
STRING(SUBSTRING ${BUILD_HASH} 6 2 HASH_B3)
STRING(SUBSTRING ${BUILD_HASH} 8 2 HASH_B4)
STRING(SUBSTRING ${BUILD_HASH} 10 2 HASH_B5)

set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/vectors.c PROPERTIES COMPILE_FLAGS "-DBUILD_HASH=\"{ 0x${HASH_B0}, 0x${HASH_B1}, 0x${HASH_B2}, 0x${HASH_B3}, 0x${HASH_B4}, 0x${HASH_B5} }\"")

ADD_LIBRARY(usercrt ${USER_RAM_SOURCES})
target_compile_definitions(usercrt PRIVATE -DCUSTOM_VERSION=${CUSTOM_VERSION})
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/common/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/Security/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/Config/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/Ram/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/Outerloop/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/Bootloader/includes" )
target_include_directories(usercrt PRIVATE "${EM718X_TOP}/libs/SensorInterface/includes" )


FUNCTION(ARC_USER_RAM_IMAGE_INTERNAL target hash cfg_file kernel)
    get_filename_component(__name ${target} NAME_WE)
    set(internal_target user-ram-${hash})
    set(${cfg}_TARGET ${internal_target} PARENT_SCOPE)
    IF(NOT TARGET ${internal_target})
        set(${cfg}_NEW_TARGET True PARENT_SCOPE)

        set(ldflags
            "-u gEM7189Config"
        )

        get_property(linker_script TARGET lib${kernel} PROPERTY USER_LINKER_SCRIPT)
        set(LINKER_SCRIPT
            ${ARC_TOOLCHAIN_LINKER_SCRIPT}"${linker_script}"
        )

        APPEND_FLAGS(ldflags
            ${LINKER_SCRIPT}
            ${USER_RAM_LD_FLAGS}
            -Wl,-e -Wl,user_init
        )

        ADD_EXECUTABLE(
            user-ram-${hash}
            ${ARGN}
        )

        TARGET_LINK_LIBRARIES(${internal_target} lib${kernel} usercrt)

        add_dependencies(${internal_target} ${USER_RAM_DEPENDS})
        target_compile_options(${internal_target} PRIVATE  ${USER_RAM_C_FLAGS})

        add_custom_command(TARGET ${internal_target} PRE_LINK
                            COMMAND ${CMAKE_COMMAND} -E remove "${CMAKE_CURRENT_BINARY_DIR}/user-ram-${hash}.usage.csv"
                            VERBATIM)

        set_property(TARGET ${internal_target} APPEND_STRING PROPERTY LINK_FLAGS " ${ldflags} ")

        add_custom_command(
            TARGET ${internal_target}
            POST_BUILD
            COMMAND ${UTILS_BIN_DIR}/initdat -q ${internal_target} ${internal_target} ${USER_RAM_INITDAT}
            DEPENDS utils
            VERBATIM
        )
    ELSE()
        # Executable already created. Re-use.
        set(${cfg}_NEW_TARGET False PARENT_SCOPE)
    ENDIF()

    if(NOT "${__name}" STREQUAL "${target}")
        add_custom_target(${__name} DEPENDS ${target})
    endif()

    IF(${SECURITY_TYPE} STREQUAL SIGNED_ALL)
        SET(USER_SIGNER_FLAGS --keyloc 2 --keyid 0)
    ELSEIF(${SECURITY_TYPE} STREQUAL SIGNED_UNSIGNED)
        SET(USER_SIGNER_FLAGS )
    ENDIF()

    IF(${ENCRYPT_USER_PAYLOAD})
        SET(ENCRYPT_FW COMMAND ${UTILS_BIN_DIR}/iencrypt -f "${__name}.fw.tmp" -o "${__name}.fw.tmp" -k "${AES_KEY}" -e)
    ELSE()
        SET(ENCRYPT_FW)
    ENDIF()

    add_custom_target(${__name}.${USER_SUFFIX} ALL DEPENDS ${target})
    add_custom_command(OUTPUT ${target}
        COMMAND ${CMAKE_COMMAND} -E copy ${internal_target} ${target}
        COMMAND ${UTILS_BIN_DIR}/stuffelf -q ${target} -f${cfg_file}
        COMMAND ${UTILS_BIN_DIR}/elf2bin ${target} ${__name}.fw.raw
        COMMAND ${UTILS_BIN_DIR}/cat "${__name}.fw.tmp" "${CMAKE_CURRENT_BINARY_DIR}/../kernel/${kernel}.${KERNEL_SUFFIX}" "${__name}.fw.raw"
        ${ENCRYPT_FW}
        COMMAND ${UTILS_BIN_DIR}/${USER_SIGNER} ${USER_SIGNER_FLAGS} --imgIdx 1 -f ${__name}.fw.tmp -o ${__name}.fw
        COMMAND ${CMAKE_COMMAND} -E remove ${__name}.fw.raw ${__name}.fw.tmp

        DEPENDS utils "${CMAKE_CURRENT_BINARY_DIR}/../kernel/${kernel}.${KERNEL_SUFFIX}" ${internal_target}
        VERBATIM
    )

    # Install fw and elf files to SDK
    INSTALL(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${__name}.fw"
            DESTINATION ${DEST_PREFIX}fw/)

    INSTALL(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${target}"
            DESTINATION ${DEST_PREFIX}elf/)

    # Additionally install fw files to fw component
    INSTALL(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${__name}.fw"
            COMPONENT ${COMPONENT_PREFIX}FW
            DESTINATION ${DEST_PREFIX}fw/)

    # Additionally install elf files to elf component
    INSTALL(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${target}"
            COMPONENT ${COMPONENT_PREFIX}ELF
            DESTINATION ${DEST_PREFIX}elf/)
ENDFUNCTION()

FUNCTION(ARC_USER_RAM_IMAGE target hash cfg_file)
    ARC_USER_RAM_IMAGE_INTERNAL(${target} ${hash} ${cfg_file} "${USER_KERNEL_IMAGE}" ${ARGN})
    set(${cfg}_TARGET ${${cfg}_TARGET} PARENT_SCOPE)
    set(${cfg}_NEW_TARGET  ${${cfg}_NEW_TARGET} PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(ARC_USER_TEST_IMAGE target hash cfg_file)
    ARC_USER_RAM_IMAGE_INTERNAL(${target} ${hash}-test ${cfg_file} "kernel-test" ${ARGN})
    set(${cfg}_TARGET ${${cfg}_TARGET} PARENT_SCOPE)
    set(${cfg}_NEW_TARGET  ${${cfg}_NEW_TARGET} PARENT_SCOPE)
ENDFUNCTION()
