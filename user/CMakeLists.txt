################################################################################
###
### @file       user/CMakeLists.txt
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
PROJECT(user C ASM)

SET(LIBS_IMPORT_FILE )

include_directories(
    includes
    ../common/includes
    ../libs/Hooks/includes
    ../libs/HostInterface/includes
    ../libs/HostInterfaceStreaming
    ../libs/SensorInterface/includes
    ../libs/Time/includes
    ../libs/I2CInterface/includes
    ../libs/Security/includes
    ../libs/Debug/includes
    ../libs/Outerloop/includes
    ../libs/Config/includes
    ../libs/Ram/includes
    ../libs/Bootloader/includes
)

add_custom_command(
    OUTPUT hooks.c
    COMMAND ${CMAKE_COMMAND} -P ${EM718X_TOP}/cmake/generate-user-hooks.cmake ${CMAKE_CURRENT_SOURCE_DIR}/../kernel/hooks.txt hooks.c user
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../kernel/hooks.txt ${EM718X_TOP}/cmake/generate-user-hooks.cmake
    VERBATIM
)

set(SOURCES main.c stack_check.c) # stack_check for missing gcc symbols
add_library(${PROJECT_NAME} OBJECT ${SOURCES} hooks.c)

message(STATUS "Linking board images against ${ROM_NAME}/${ROM_IMAGE}")

SET(HASH_LIST )
foreach(b ${BOARDS})
    set(cfg "${EM718X_TOP}/boards/${b}.cfg")
    set(board_name ${b})

    READ_BOARD_CONFIG(${cfg})
    SET(config_specs ${${cfg}_CONFIG_SPEC})
    SET(build_ram ${${cfg}_BUILD_RAM})
    SET(build_test ${${cfg}_BUILD_TEST})
    SET(build_flash ${${cfg}_BUILD_FLASH})
    SET(physical_driver_ids ${${cfg}_PHYSICAL_DRIVER_IDS})
    SET(physical_driver_names ${${cfg}_PHYSICAL_DRIVER_LIBS})
    SET(virtual_driver_ids ${${cfg}_VIRTUAL_DRIVER_IDS})
    SET(virtual_driver_names ${${cfg}_VIRTUAL_DRIVER_LIBS})
    SET(board_specific_libs ${${cfg}_BOARD_LIBS})
    SET(board_specific_ram_patches ${${cfg}_BOARD_PATCHES})
    SET(boad_ram_patch_library RamPatches)

    set(drivers ${physical_driver_names} ${virtual_driver_names})
    # message(STATUS "Board ${b} requires drivers ${drivers}")

    # Generate a unique id based on anything that can change the output of the linker
    STRING(MD5 elf_hash "${drivers};${config_specs};${board_specific_libs};${board_specific_ram_patches}" )

    message(STATUS "Board ${b} (hash: ${elf_hash})")
    LIST(APPEND HASH_LIST "${elf_hash}: ${b}.cfg")

    LIST(LENGTH config_specs len)
    IF(${len})
        SET(RELINK_FILE ${CMAKE_CURRENT_BINARY_DIR}/config_strings_${b}.c)
        GENERATE_CONFIG_STRING_FILE(${RELINK_FILE} config_specs)
    ELSE()
        SET(RELINK_FILE ${CMAKE_CURRENT_BINARY_DIR}/relink_${b}.c)
        FILE(WRITE ${RELINK_FILE} "")
    ENDIF()



    ### Add a dependency on the .cfg file.
    configure_file(${cfg} ${CMAKE_CURRENT_BINARY_DIR}/${b}.cfg COPYONLY)  ## Notify the cmake system that it *must* re-build the cache if the .cfg file changes.
    SET_SOURCE_FILES_PROPERTIES(
        ${RELINK_FILE} PROPERTIES OBJECT_DEPENDS ${cfg}
    )

    LIST(LENGTH board_specific_ram_patches patch_len)
    IF(${patch_len})
        SET(boad_ram_patch_library ${elf_hash}-RamPatches)
    ENDIF()

    if(NOT TARGET ${boad_ram_patch_library})
        # Add the board specific RamPatch as separate binary directory
        add_subdirectory(RamPatches ${boad_ram_patch_library})
    endif()

    IF(build_ram)
        ## Ram Image
        set(elf ${b}.elf)
        ARC_USER_RAM_IMAGE(${elf} ${elf_hash} ${cfg} $<TARGET_OBJECTS:${PROJECT_NAME}> ${RELINK_FILE})
        set(target ${${cfg}_TARGET})
        set(new_target ${${cfg}_NEW_TARGET})
        IF(${new_target})
            ARC_LINK_LIBRARIES(${target} ${drivers} ${board_specific_libs} ${boad_ram_patch_library} ${BOARDS_LIBS})
        ENDIF()
    ENDIF()

    IF(build_flash)
        ### Flash Image
        set(b ${board_name}-flash)
        set(flashelf ${b}.elf)
        ARC_USER_FLASH_IMAGE(${flashelf} ${elf_hash} ${cfg} $<TARGET_OBJECTS:${PROJECT_NAME}> ${RELINK_FILE})
        set(target ${${cfg}_TARGET})
        set(new_target ${${cfg}_NEW_TARGET})
        IF(${new_target})
            ARC_LINK_LIBRARIES(${target} ${drivers} ${board_specific_libs} ${boad_ram_patch_library}-flash ${BOARDS_LIBS})
        ENDIF()
    ENDIF()

    IF(build_test)
        ## Test Image
        set(b ${board_name}-test)
        set(elf ${b}.elf)
        ARC_USER_TEST_IMAGE(${elf} ${elf_hash} ${cfg} $<TARGET_OBJECTS:${PROJECT_NAME}> ${RELINK_FILE})
        set(target ${${cfg}_TARGET})
        set(new_target ${${cfg}_NEW_TARGET})
        IF(${new_target})
            ARC_LINK_LIBRARIES(${target} ${drivers} ${board_specific_libs} ${boad_ram_patch_library} ${BOARDS_LIBS})
        ENDIF()
    ENDIF()
endforeach()

LIST(SORT HASH_LIST)
SET(PRINT_CMDS )
SET(FILE_CONTENTS )
foreach(item ${HASH_LIST})
    LIST(APPEND FILE_CONTENTS "${item}\n")
    LIST(APPEND PRINT_CMDS COMMAND ${CMAKE_COMMAND} -E echo ${item})
endforeach()

FILE(WRITE ${CMAKE_BINARY_DIR}/board_mapping.txt ${FILE_CONTENTS})
add_custom_target(print-boards
                ${PRINT_CMDS}
                VERBATIM
                USES_TERMINAL
                )

IF(NOT SDK)
    install(FILES CMakeLists.txt DESTINATION ${PROJECT_NAME})
    install(FILES ${SOURCES} DESTINATION ${PROJECT_NAME})
    install(FILES ${HEADERS} DESTINATION ${PROJECT_NAME})
ENDIF()
