################################################################################
###
### @file       common/macros.cmake
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
INCLUDE(CMakeParseArguments)

IF(EM718X_TOP)
    include("${EM718X_TOP}/cmake/expand_command.cmake")
    include("${EM718X_TOP}/cmake/hex2dec.cmake")
ENDIF()



MACRO(ADD_C_FLAGS)
    FOREACH(flag ${ARGN})
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flag}")
    ENDFOREACH(flag)
ENDMACRO(ADD_C_FLAGS)


MACRO(ADD_CXX_FLAGS)
    FOREACH(flag ${ARGN})
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
    ENDFOREACH(flag)
ENDMACRO()


MACRO(REMOVE_C_FLAGS)
    FOREACH(flag ${ARGN})
        STRING(REPLACE "${flag}" "" CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
    ENDFOREACH(flag)

    STRING(REPLACE "  " " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
    STRING(STRIP ${CMAKE_C_FLAGS} CMAKE_C_FLAGS)
ENDMACRO(REMOVE_C_FLAGS)


MACRO(ADD_ASM_FLAGS)
    FOREACH(flag ${ARGN})
        SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${flag}")
    ENDFOREACH(flag)
ENDMACRO(ADD_ASM_FLAGS)


MACRO(ADD_FLAGS)
    ADD_ASM_FLAGS(${ARGN})
    ADD_C_FLAGS(${ARGN})
    ADD_CXX_FLAGS(${ARGN})
ENDMACRO()


MACRO(REMOVE_ASM_FLAGS)
    FOREACH(flag ${ARGN})
        STRING(REPLACE "${flag}" "" CMAKE_ASM_FLAGS ${CMAKE_ASM_FLAGS})
    ENDFOREACH(flag)

    STRING(REPLACE "  " " " CMAKE_ASM_FLAGS ${CMAKE_ASM_FLAGS})
    STRING(STRIP ${CMAKE_ASM_FLAGS} CMAKE_ASM_FLAGS)
ENDMACRO(REMOVE_ASM_FLAGS)


MACRO(ADD_LD_FLAGS)
    FOREACH(flag ${ARGN})
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${flag}")
    ENDFOREACH(flag)
ENDMACRO(ADD_LD_FLAGS)


MACRO(REMOVE_LD_FLAGS)
    FOREACH(flag ${ARGN})
        STRING(REPLACE "${flag}" "" CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})
    ENDFOREACH(flag)

    STRING(REPLACE "  " " " CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})
    STRING(STRIP ${CMAKE_EXE_LINKER_FLAGS} CMAKE_EXE_LINKER_FLAGS)
ENDMACRO(REMOVE_LD_FLAGS)

FUNCTION(ARC_LINK_LIBRARIES target)
    SET(_ARC_LINK_PATH "")

    foreach(lib ${ARGN})
        target_link_libraries(${target} ${lib})
        SET(_ARC_LINK_PATH ${_ARC_LINK_PATH} $<TARGET_FILE:${lib}>)

        list(APPEND _libs $<TARGET_FILE:${lib}>)

        get_property(interfaces TARGET ${lib} PROPERTY INTERFACE_LINK_LIBRARIES)
        foreach(interface ${interfaces})
            list(APPEND _libs $<TARGET_FILE:${interface}>)

            SET(_ARC_LINK_PATH ${_ARC_LINK_PATH} $<TARGET_FILE:${interface}>)
        endforeach()
    endforeach()

    LIST(REMOVE_DUPLICATES _libs)

    set(_d "${CMAKE_CURRENT_BINARY_DIR}/${target}.data" )
    FILE(GENERATE OUTPUT ${_d}/libs CONTENT "${_libs}" )
    FILE(GENERATE OUTPUT ${_d}/sections CONTENT "${INCLUDE_SECTIONS}" )

    add_custom_command(TARGET ${target} POST_BUILD
                       COMMAND "${CMAKE_COMMAND}" -P "${EM718X_TOP}/cmake/allocations.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${target}.usage.csv" ${_d}/libs
                       DEPENDS ${_d}/libs
                       VERBATIM)

    add_custom_command(TARGET ${target} PRE_LINK
        COMMAND "${CMAKE_COMMAND}" -P "${EM718X_TOP}/cmake/force_include.cmake" "${USE_GCC}" "${CMAKE_CURRENT_BINARY_DIR}/${target}.lib.inc" ${_d}/libs ${_d}/sections
        DEPENDS ${_d}/libs ${_d}/sections
        BYPRODUCTS ${target}.lib.inc
        VERBATIM
    )

    set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " @\"${CMAKE_CURRENT_BINARY_DIR}/${target}.lib.inc\"")
ENDFUNCTION(ARC_LINK_LIBRARIES)

MACRO(APPEND_FLAGS var)
    foreach(f ${ARGN})
        set(${var} "${${var}} ${f}")
    endforeach()
ENDMACRO()

FUNCTION(ARC_IMPORTED_ROM_IMAGE target location)
    get_filename_component(__name ${target} NAME_WE)
    get_filename_component(__src_dir ${location} DIRECTORY)
    get_filename_component(__dir ${ROM_PATH} DIRECTORY)

    add_custom_command(
                OUTPUT ${ROM_PATH}
                COMMAND ${CMAKE_COMMAND} -E copy "${location}" ${ROM_PATH}
                COMMAND ${CMAKE_COMMAND} -E copy "${location}.used_ram" ${ROM_PATH}.used_ram
                COMMAND ${CMAKE_COMMAND} -E copy "${location}.reclaim_ram" ${ROM_PATH}.reclaim_ram
                COMMAND ${CMAKE_COMMAND} -E copy ${__src_dir}/${__name}.hex ${__dir}/${__name}.hex
                COMMAND ${CMAKE_COMMAND} -E copy ${__src_dir}/${__name}.otp.hex ${__dir}/${__name}.otp.hex
                DEPENDS ${location}
                VERBATIM
                )

    add_custom_command(OUTPUT ${ROM_PATH}.used_ram DEPENDS ${ROM_PATH})
    add_custom_command(OUTPUT ${ROM_PATH}.reclaim_ram DEPENDS ${ROM_PATH})
    add_custom_command(OUTPUT ${__dir}/${__name}.hex DEPENDS ${ROM_PATH})
    add_custom_command(OUTPUT ${__dir}/${__name}.otp.hex DEPENDS ${ROM_PATH})


    add_custom_target(rom DEPENDS ${ROM_PATH})
    set_target_properties(rom PROPERTIES LOCATION "${location}")

    add_custom_target(jli DEPENDS ${JLI})
    add_custom_command(
        OUTPUT ${JLI}
        COMMAND ${CMAKE_COMMAND} -P "${EM718X_TOP}/cmake/generate-jli.cmake" "${location}" ${JLI_SECTION} ${JLI}
        DEPENDS "${location}"
        COMMENT "generating ${JLI}"
        VERBATIM
    )

    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${__name}.s"
        COMMAND ${CMAKE_COMMAND} -P "${EM718X_TOP}/cmake/generate-symasm.cmake" "${location}" "${CMAKE_CURRENT_BINARY_DIR}/${__name}.s" $<TARGET_PROPERTY:rom,WHITELIST> $<TARGET_PROPERTY:rom,BLACKLIST> $<TARGET_PROPERTY:rom,FORCE_EXPORT>
        DEPENDS ${location} $<TARGET_PROPERTY:rom,WHITELIST> $<TARGET_PROPERTY:rom,BLACKLIST> $<TARGET_PROPERTY:rom,FORCE_EXPORT>
        VERBATIM
    )

    set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/${__name}.s" PROPERTIES COMPILE_FLAGS -UVERSION)
    add_library(lib${__name} "${CMAKE_CURRENT_BINARY_DIR}/${__name}.s")
ENDFUNCTION()

FUNCTION(ARC_LINK_ROM target)
    set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " @\"${ROM_PATH}.used_ram\" ")
    set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " @\"${ROM_PATH}.reclaim_ram\" ")
    target_link_libraries(${target} lib${ROM_NAME})
ENDFUNCTION()

MACRO(SET_DRIVERS)
    SET(_drivers ${ENABLED_DRIVERS})

    foreach(d ${_drivers})
        message(STATUS "Boards require driver: ${d}")
    endforeach()

    foreach(d ${DRIVERS_EXTRA})
        message(STATUS "Adding extra driver: ${d}")
        list(APPEND _drivers ${d})
    endforeach()

    SET(DRIVERS ${_drivers})
ENDMACRO()

# This macro reads in the board cfg file and pulls out any information that the build system needs.
FUNCTION(READ_BOARD_CONFIG cfg)
    ## Search for configuration strings for the board file
    get_filename_component(_board ${cfg} NAME_WE)

    FILE(STRINGS ${cfg} config_list REGEX config_list,)
    STRING(REPLACE "config_list," "" config_list "${config_list}")

    LIST(LENGTH config_list len)
    IF(${len})
        set(config_spec )
        FOREACH(__path ${config_list})
            MESSAGE(" -- Reading in configuration list from ${__path}")
            get_filename_component(__dir ${__path} DIRECTORY)
            set(__path "${EM718X_TOP}/${__path}")

            if(NOT EXISTS ${__path})
                message(FATAL_ERROR "board file ${cfg} requires cfg spec list ${__path} that does not exist")
            endif()

            FILE(STRINGS ${__path} list_specs)
            FOREACH(__spec ${list_specs})
                set(config_spec ${config_spec} ${__dir}/../spec/${__spec})
            ENDFOREACH()
        ENDFOREACH()
    ELSE()
        #User specified the configuration strings manually.
        FILE(STRINGS ${cfg} config_spec REGEX config_spec,)
        STRING(REPLACE "config_spec," "" config_spec "${config_spec}")
    ENDIF()

    ## Determine the type of firmware to build (flash, ram, or all)
    FILE(STRINGS ${cfg} build_type REGEX build_type,)
    LIST(LENGTH build_type len)
    IF(${len} EQUAL 1)
        STRING(REPLACE "build_type," "" build_type "${build_type}")
        if(build_type STREQUAL "ram")
            # Ram only
            set(${cfg}_BUILD_RAM True PARENT_SCOPE)
            set(${cfg}_BUILD_FLASH False PARENT_SCOPE)
            set(${cfg}_BUILD_TEST False PARENT_SCOPE)
        ELSEIF(build_type STREQUAL "flash")
            # Flash only
            set(${cfg}_BUILD_RAM False PARENT_SCOPE)
            set(${cfg}_BUILD_FLASH True PARENT_SCOPE)
            set(${cfg}_BUILD_TEST False PARENT_SCOPE)
        ELSEIF(build_type STREQUAL "test")
            # Test only
            set(${cfg}_BUILD_RAM False PARENT_SCOPE)
            set(${cfg}_BUILD_FLASH False PARENT_SCOPE)
            set(${cfg}_BUILD_TEST True PARENT_SCOPE)
        ELSE()
            # Default to ram/flash.
            set(${cfg}_BUILD_RAM True PARENT_SCOPE)
            set(${cfg}_BUILD_FLASH True PARENT_SCOPE)
            set(${cfg}_BUILD_TEST False PARENT_SCOPE)
        ENDIF()
    ELSE()
        set(${cfg}_BUILD_RAM True PARENT_SCOPE)
        set(${cfg}_BUILD_FLASH True PARENT_SCOPE)
        set(${cfg}_BUILD_TEST False PARENT_SCOPE)
    ENDIF()
    IF(NOT BUILD_FLASH)
        set(${cfg}_BUILD_FLASH False PARENT_SCOPE)
    ENDIF()

    ## Determine any additional libraries needed
    FILE(STRINGS ${cfg} libs REGEX lib,)
    STRING(REPLACE "lib," "" libs "${libs}")
    STRING(REPLACE "," ";" libs "${libs}")
    FOREACH(__lib ${libs})
       MESSAGE(" -- Enabling library ${__lib}")
    ENDFOREACH()
    set(${cfg}_BOARD_LIBS ${libs} PARENT_SCOPE)

    ## Determine if any board specific ram patches needed
    FILE(STRINGS ${cfg} patches REGEX ram_patches,)
    STRING(REPLACE "ram_patches," "" patches "${patches}")
    STRING(REPLACE "," ";" patches "${patches}")
    FOREACH(__patch ${patches})
       MESSAGE(" -- Enabling user ram_patch ${__patch}")
    ENDFOREACH()
    set(${cfg}_BOARD_PATCHES ${patches} PARENT_SCOPE)

    FOREACH(__path ${config_spec})
        set(__path "${EM718X_TOP}/${__path}")
        MESSAGE(" -- Reading in configuration file from ${__path}")
        if(NOT EXISTS ${__path})
            message(FATAL_ERROR "board file ${cfg} requires cfg spec ${__path} that does not exist")
        endif()
    ENDFOREACH()
    set(${cfg}_CONFIG_SPEC ${config_spec} PARENT_SCOPE)

    # get list of physical drivers
    # eg: a45,spi0,25,2, 1, 0, 0, 0,-1, 0, 0, 0,-1, 0, 0, 0, -1.000000, 0
    SET(__driver_regex "^[agm]?([0-9]+),[a-zA-Z0-9]+,[0-9]+,.*")
    FILE(STRINGS ${cfg} __drv REGEX ${__driver_regex})
    FOREACH(__line ${__drv})
        STRING(REGEX MATCH ${__driver_regex} __m ${__line} )
        #message(STATUS "${_board} PHYS: ${__line}  ID: ${CMAKE_MATCH_1}")
        SET(__id "${CMAKE_MATCH_1}")
        set(__lib "${DRIVER_${__id}}")
        if(NOT __lib)
            message(FATAL_ERROR "BOARD ${_board} REQUIRES INVALID DRIVER ID ${__id}")
        ENDIF()
        list(APPEND __ids "${__id}" )
        list(APPEND __libs "${__lib}")
    ENDFOREACH()
    list(REMOVE_DUPLICATES __libs)

    SET(${cfg}_PHYSICAL_DRIVER_IDS "${__ids}" PARENT_SCOPE)
    SET(${cfg}_PHYSICAL_DRIVER_LIBS "${__libs}" PARENT_SCOPE)

    set(__ids)
    set(__libs)

    # find virtual drivers:
    # eg: 216, -1.000000
    FILE(STRINGS ${cfg} __drv1 REGEX "^[ \t]*[agm]?([0-9]+),[ \t]*[-.0-9]+[ \t]*$")
    # eg: 216, -1.000000 # gas depends on a physical gas source.
    FILE(STRINGS ${cfg} __drv2 REGEX "^[ \t]*[agm]?([0-9]+),[ \t]*[-.0-9]+[ \t]+#.*$")
    lIST(APPEND __drv ${__drv1} ${__drv2})
    FOREACH(__line ${__drv})
        STRING(REGEX MATCH "^[ \t]*[agm]?([0-9]+),.*$" __m ${__line} )
        #message(STATUS "${_board} VIRT: ${__line}  ID: ${CMAKE_MATCH_1}")
        SET(__id "${CMAKE_MATCH_1}")
        set(__lib "${DRIVER_${__id}}")
        if(NOT __lib)
            message(WARNING "BOARD ${_board} CONFIG REQUESTED INVALID DRIVER ID ${__id}")
        ENDIF()
        list(APPEND __ids "${__id}" )
        list(APPEND __libs "${__lib}")
    ENDFOREACH()
    list(REMOVE_DUPLICATES __libs)

    SET(${cfg}_VIRTUAL_DRIVER_IDS "${__ids}" PARENT_SCOPE)
    SET(${cfg}_VIRTUAL_DRIVER_LIBS "${__libs}" PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(GENERATE_CONFIG_STRING_FILE file config_strings)
    set(__list ${${config_strings}})
    LIST(REMOVE_DUPLICATES __list)
    LIST(LENGTH __list len)

    FILE(WRITE ${file}  "// Autogenerated file, do not modify\n"
                        "#include <types.h>\n\n"
    )

    FOREACH(__path ${__list})
        set(name ${__path})
        STRING(REPLACE "." "__" name ${name})
        STRING(REPLACE "/" "__" name ${name})
        FILE(APPEND ${file}
            "static UInt8 RECLAIM_DATA config_spec_${name}[] = {\n"
            "   #include <${EM718X_TOP}/${__path}>\n"
            "};\n"
        )
    ENDFOREACH()


    FILE(APPEND ${file} "\n"
                        "UInt8 RECLAIM_DATA g_num_config_strings = ${len};\n")

    FILE(APPEND ${file} "UInt8 RECLAIM_DATA *g_config_strings[] = {\n")
    FOREACH(__path ${__list})
        set(name ${__path})
        STRING(REPLACE "." "__" name ${name})
        STRING(REPLACE "/" "__" name ${name})
        FILE(APPEND ${file}
            "   config_spec_${name},\n"
        )
    ENDFOREACH()
    FILE(APPEND ${file} "};\n")

    FILE(APPEND ${file} "UInt32 RECLAIM_DATA g_config_sizes[] = {\n")
    FOREACH(__path ${__list})
        set(name ${__path})
        STRING(REPLACE "." "__" name ${name})
        STRING(REPLACE "/" "__" name ${name})
        FILE(APPEND ${file}
            "   sizeof(config_spec_${name}),\n"
        )
    ENDFOREACH()
    FILE(APPEND ${file} "};\n")

ENDFUNCTION()


FUNCTION(ADD_ARC_DRIVER_INTERNAL target driver_id)
    ADD_LIBRARY(${target} ${ARGN})

    TARGET_COMPILE_OPTIONS(${target} PRIVATE -DDRIVER_ID=${driver_id})
    set_target_properties(${target} PROPERTIES
        DRIVER_ID ${driver_id}
    )

    # Error out if the driver was already loaded with a different name.
    # Note: this will fail if a driver is re-named unless if the cache is cleared.
    if(DRIVER_${driver_id} AND NOT "${DRIVER_${driver_id}}" STREQUAL "${target}")
        MESSAGE(FATAL_ERROR
            "DRIVER_ID ${driver_id} already specified by ${DRIVER_${driver_id}}. Unable to add ${target}"
            )
    endif()

    SET("DRIVER_${driver_id}" "${target}" CACHE INTERNAL "" )
ENDFUNCTION(ADD_ARC_DRIVER_INTERNAL)

FUNCTION(ADD_ARC_DRIVER target driver_id)
    get_filename_component(driver_absdir ${CMAKE_CURRENT_LIST_DIR}/../ ABSOLUTE)
    get_filename_component(driver_dir ${driver_absdir} NAME)
    ADD_ARC_DRIVER_INTERNAL(${target} ${driver_id} ${ARGN})

    LIST(FIND DRIVERS_NO_SOURCE ${target} binary_only)
    if(NOT ${binary_only} EQUAL -1)
        set(_source_msg "(installing binaries only)")
        ## Driver can only be distributed in object form.
        install(FILES $<TARGET_FILE:${target}>
                DESTINATION ${driver_dir}/${target})

        set(LIBNAME ${target})
        set(DRIVER_ID ${driver_id})
        configure_file("${EM718X_TOP}/${driver_dir}/CMakeLists.no-source.in" "${CMAKE_CURRENT_BINARY_DIR}/CMakeLists.txt.sdk" @ONLY)
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/CMakeLists.txt.sdk"
                DESTINATION ${driver_dir}/${target}
                RENAME CMakeLists.txt)
    else()
        IF(NOT SDK)
            set(_source_msg "(installing source)")
            FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
            ## Driver can be distributed in source form.
            install(FILES ${ARGN} ${headers}
                    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
                    DESTINATION ${driver_dir}/${target})
        ENDIF()
    endif()

    message(STATUS "Building driver ${target} ${_source_msg}")
ENDFUNCTION(ADD_ARC_DRIVER)

FUNCTION(ADD_IMPORTED_INTERNAL name location)
    message(STATUS "Add imported library ${name} at ${CMAKE_CURRENT_LIST_DIR}")
    add_library(${name} IMPORTED STATIC GLOBAL)
    set_target_properties(${name} PROPERTIES
        IMPORTED_LOCATION ${location}
    )

    if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/includes)
        set_target_properties(${name} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/includes"
        )
        set_target_properties(${name} PROPERTIES
            INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/includes"
        )
    endif()
ENDFUNCTION(ADD_IMPORTED_INTERNAL)

FUNCTION(ADD_IMPORTED_LIBRARY name location)
    ADD_IMPORTED_INTERNAL(${name} ${location})

    EXPORT_LIBRARY_BINARY(${name})
    EXPORT_LIBRARY_HEADERS(${name})
ENDFUNCTION(ADD_IMPORTED_LIBRARY)

FUNCTION(ADD_IMPORTED_DRIVER name driver_id location)
    get_filename_component(driver_absdir ${CMAKE_CURRENT_LIST_DIR}/../ ABSOLUTE)
    ADD_IMPORTED_INTERNAL(${name} ${location})

    set_target_properties(${name} PROPERTIES
        DRIVER_ID ${driver_id}
    )

    # Error out if the driver was already loaded with a different name.
    # Note: this will fail if a driver is re-named unless if the cache is cleared.
    if(DRIVER_${driver_id} AND NOT "${DRIVER_${driver_id}}" STREQUAL "${name}")
        MESSAGE(FATAL_ERROR
            "DRIVER_ID ${driver_id} already specified by ${DRIVER_${driver_id}}. Unable to add ${name}"
            )
    endif()

    SET("DRIVER_${driver_id}" "${name}" CACHE INTERNAL "" )

    EXPORT_DRIVER_BINARY(${name})
ENDFUNCTION(ADD_IMPORTED_DRIVER)

FUNCTION(INSTALL_EXISTS path install)
    if(NOT SDK AND IS_DIRECTORY ${path})
        install(DIRECTORY ${path} DESTINATION ${install} FILES_MATCHING PATTERN "*.h")
    endif()
ENDFUNCTION(INSTALL_EXISTS)

FUNCTION(ARC_INSTALL_LIBRARY_HEADERS name folder)
    IF(SDK)
        RETURN()
    ENDIF()

    # Allow additional headers to be installed only if the library is an exported library.
    SET(EXPORTS ${EXPORT_LIB_SOURCE} ${EXPORT_LIB_HEADERS} ${EXPORT_LIB_BINARIES})
    LIST(FIND EXPORTS ${name} _found)
    IF(${_found} EQUAL -1)
        RETURN()
    ENDIF()

    set(LIBNAME ${name})
    IF(TARGET ${name}-link)
        SET(FILENAME ${name}-link)
    ELSE()
        SET(FILENAME ${name})
    ENDIF()

    INSTALL_EXISTS(${CMAKE_CURRENT_LIST_DIR}/${folder}/ libs/${name}/${folder})
    set_property(TARGET ${LIBNAME} APPEND PROPERTY LIB_INSTALL_HEADERS ${folder})
    get_target_property(LIBHEADERS ${LIBNAME} LIB_INSTALL_HEADERS)
    configure_file("${EM718X_TOP}/libs/CMakeLists.no-source.in" "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" @ONLY)
    install(FILES "${EM718X_TOP}/libs/CMakeLists.no-source.in" DESTINATION libs/)
ENDFUNCTION(ARC_INSTALL_LIBRARY_HEADERS)

FUNCTION(ARC_EXPORT_STAGE stage path)
    get_filename_component(__name ${stage} NAME_WE)
    install(FILES $<TARGET_FILE:lib${__name}>
            DESTINATION ${path})
ENDFUNCTION(ARC_EXPORT_STAGE)

FUNCTION(ADD_DUMMY_LIB name)
    SET(DUMMY_FILE "${CMAKE_CURRENT_BINARY_DIR}/${name}.c")
    add_library(${name} STATIC ${DUMMY_FILE})
    FILE(WRITE ${DUMMY_FILE} "")
ENDFUNCTION(ADD_DUMMY_LIB)

FUNCTION(EXPORT_LIBRARY_SOURCE name)
    IF(SDK)
        RETURN()
    ENDIF()

    # Source is allowed only if the library is in EXPORT_LIB_SOURCE
    LIST(FIND EXPORT_LIB_SOURCE ${name} _found)
    IF(${_found} EQUAL -1)
        RETURN()
    ENDIF()

    MESSAGE(" -- Installing sources")
    FOREACH(file ${ARGN})
        # Install files while keeping the directory structure.
        get_source_file_property(file ${file} LOCATION)
        get_filename_component(path ${file} DIRECTORY)
        FILE(RELATIVE_PATH path ${CMAKE_CURRENT_LIST_DIR} ${path})
        install(FILES ${file} DESTINATION libs/${name}/${path})
    ENDFOREACH()
    install(FILES ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt DESTINATION libs/${name})
    # install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ DESTINATION libs/${name} )
ENDFUNCTION()

FUNCTION(BUILD_LIBRARY_BINARY name)
    # Binary is not generated if we only need headers.
    LIST(FIND BUILD_LIBS ${name} _found)
    IF(${_found} EQUAL -1)
        RETURN()
    ENDIF()

    MESSAGE(" -- Building library")
    add_library(${name} STATIC ${ARGN})

    if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/includes)
        target_include_directories(${name} PUBLIC
                                    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/includes>
                                    $<INSTALL_INTERFACE:libs/${name}/includes>
                                    )
    endif()
ENDFUNCTION(BUILD_LIBRARY_BINARY)


FUNCTION(EXPORT_LIBRARY_BINARY name)
    IF(SDK)
        RETURN()
    ENDIF()

    # Binary is allowed only if the library is in EXPORT_LIB_BINARIES
    LIST(FIND EXPORT_LIB_BINARIES ${name} _found)
    IF(${_found} EQUAL -1)
        RETURN()
    ENDIF()

    MESSAGE(" -- Installing binary and headers to ${CMAKE_CURRENT_BINARY_DIR}")
    install(FILES $<TARGET_FILE:${name}>
            DESTINATION libs/${name})

    set(LIBNAME ${name})
    set(FILENAME ${name})
    configure_file("${EM718X_TOP}/libs/CMakeLists.no-source.in" "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" DESTINATION libs/${name})
    install(FILES "${EM718X_TOP}/libs/CMakeLists.txt.sdk" DESTINATION libs/${name} RENAME CMakeLists.txt)
    install(FILES "${EM718X_TOP}/libs/CMakeLists.no-source.in" DESTINATION libs/)
    INSTALL_EXISTS(${CMAKE_CURRENT_LIST_DIR}/includes/ libs/${name}/includes)
ENDFUNCTION()

FUNCTION(EXPORT_LIBRARY_HEADERS name)
    IF(SDK)
        RETURN()
    ENDIF()

    # Headers are allowed if the library is in EXPORT_LIB_HEADERS
    LIST(FIND EXPORT_LIB_HEADERS ${name} _found)
    IF(${_found} EQUAL -1)
        RETURN()
    ENDIF()

    MESSAGE(" -- Installing header")
        # Create a dummy library to satisfy target_link_libarary() calls to this lib if needed.
        IF(TARGET ${name})
            SET(DUMMY_LIB ${name}-link)
        ELSE()
            SET(DUMMY_LIB ${name})
        ENDIF()

        ADD_DUMMY_LIB(${DUMMY_LIB})

        install(FILES $<TARGET_FILE:${DUMMY_LIB}>
                DESTINATION libs/${name})

        if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/includes)
            target_include_directories(${name} INTERFACE
                                        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/includes>
                                        $<INSTALL_INTERFACE:libs/${name}/includes>
                                        )

            install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/includes/ DESTINATION libs/${name}/includes )
        endif()

        set(LIBNAME ${name})
        set(FILENAME ${DUMMY_LIB})
        configure_file("${EM718X_TOP}/libs/CMakeLists.no-source.in" "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" @ONLY)
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" DESTINATION libs/${name})
        install(FILES "${EM718X_TOP}/libs/CMakeLists.txt.sdk" DESTINATION libs/${name} RENAME CMakeLists.txt)
        install(FILES "${EM718X_TOP}/libs/CMakeLists.no-source.in" DESTINATION libs/)
ENDFUNCTION()

FUNCTION(ADD_ARC_LIBRARY name)
    MESSAGE(STATUS "Adding Library ${name}")
    BUILD_LIBRARY_BINARY(${name} ${ARGN})
    EXPORT_LIBRARY_SOURCE(${name} ${ARGN})
    EXPORT_LIBRARY_BINARY(${name} ${ARGN})
    EXPORT_LIBRARY_HEADERS(${name} ${ARGN})
ENDFUNCTION(ADD_ARC_LIBRARY)

FUNCTION(EXPORT_DRIVER_BINARY name)
    IF(SDK)
        RETURN()
    ENDIF()

    MESSAGE(" -- Installing binary and headers")
    install(FILES $<TARGET_FILE:${name}>
            DESTINATION drivers/${name})

    set(LIBNAME ${name})
    SET(DRIVER_ID ${driver_id})
    configure_file("${EM718X_TOP}/drivers/CMakeLists.no-source.in" "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}.sdk.cmake" DESTINATION drivers/${name})
    install(FILES "${EM718X_TOP}/drivers/CMakeLists.txt.sdk" DESTINATION drivers/${name} RENAME CMakeLists.txt)
    install(FILES "${EM718X_TOP}/drivers/CMakeLists.no-source.in" DESTINATION drivers/)
    INSTALL_EXISTS(${CMAKE_CURRENT_LIST_DIR}/includes/ drivers/${name}/includes)
ENDFUNCTION()


MACRO(EXPORT_ARC_LIBRARY target)
#    install(EXPORT ${target} DESTINATION libs/${target})
ENDMACRO()


MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(APPEND var suffix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${f}${suffix}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(APPEND)

MACRO(SHOW_ALL_VARS)
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
ENDMACRO()


MACRO(GET_SECTION_SIZE VAR elf section)

    EXECUTE_PROCESS(COMMAND nmac -dq ${elf} OUTPUT_VARIABLE nmac)

    SET(start_regex ".*_f${section}$")
    SET(end_regex ".*_e${section}$")
    SET(end "")
    SET(start "")

    STRING(REGEX REPLACE "\n" ";" nmac "${nmac}")
    FOREACH(line as ${nmac})
        STRING(REGEX MATCHALL "${start_regex}" found ${line})
        IF(NOT "${found}" STREQUAL "")
            STRING(STRIP ${found} found)
            STRING(REGEX REPLACE "[ \t]+" ";" words "${found}")
            LIST(GET words 0 start)
        ENDIF(NOT "${found}" STREQUAL "")

        STRING(REGEX MATCHALL "${end_regex}" found ${line})
        IF(NOT "${found}" STREQUAL "")
            STRING(STRIP ${found} found)
            STRING(REGEX REPLACE "[ \t]+" ";" words "${found}")
            LIST(GET words 0 end)
        ENDIF(NOT "${found}" STREQUAL "")
    ENDFOREACH()

    IF(NOT "${end}" STREQUAL "" AND NOT "${start}" STREQUAL "")
        math(EXPR size "${end} - ${start}")
    ELSE(NOT "${end}" STREQUAL "" AND NOT "${start}" STREQUAL "")
        SET(size 0)
    ENDIF(NOT "${end}" STREQUAL "" AND NOT "${start}" STREQUAL "")

ENDMACRO()

FUNCTION(MAKE_DEBUG_ELF name debug_exports)
    set(EXPORT_S_FILE ${name}_${debug_exports}.s)
    set(DEBUG_ELF ${name}_debug.elf)
    MESSAGE("Creating ${DEBUG_ELF} debug file")
    add_custom_command(
        OUTPUT ${EXPORT_S_FILE}
        POST_BUILD
        DEPENDS "${EM718X_TOP}/build/kernel/${name}.elf"
        DEPENDS "${EM718X_TOP}/kernel/${debug_exports}"
        COMMAND ${CMAKE_COMMAND} -P "${EM718X_TOP}/cmake/generate-symasm.cmake" "${EM718X_TOP}/build/kernel/${name}.elf" ${EXPORT_S_FILE} "${EM718X_TOP}/kernel/${debug_exports}" "" ""
        VERBATIM
    )
    add_executable(${DEBUG_ELF} ${EXPORT_S_FILE})
    set_property(TARGET ${DEBUG_ELF} PROPERTY LINK_FLAGS " -emain")
    set_property(TARGET ${DEBUG_ELF} APPEND_STRING PROPERTY LINK_FLAGS " \"${ARC_TOOLCHAIN_LINKER_SCRIPT}${EM718X_TOP}/kernel/debug_link.cmd\" ")
    set_property(TARGET ${DEBUG_ELF} PROPERTY FLAGS "")

    IF(GENERATES_SDK)
        INSTALL(FILES
                "${CMAKE_CURRENT_BINARY_DIR}/${DEBUG_ELF}"
                DESTINATION elf/)
    ENDIF()
ENDFUNCTION()


FUNCTION(ADD_PRE_TEST name)
    IF(NOT TARGET pre_test)
        # Add a top level target to run before anything build.
        add_custom_target(pre_test ALL)
    ENDIF()

    add_dependencies(pre_test ${name})

ENDFUNCTION(ADD_PRE_TEST)

FUNCTION(ADD_POST_TEST name)
    IF(NOT TARGET post_test)
        # Add a top level target to run after evertyhing.
        # This does not explictly add dependencies, so any jobs triggered by it should mark all needed dependencies.
        add_custom_target(post_test ALL)
    ENDIF()

    add_dependencies(post_test ${name})
ENDFUNCTION(ADD_POST_TEST)
