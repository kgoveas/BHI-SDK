set(CUSTOM_VERSION 0)
set(BUILD_YEAR 2021)
set(BUILD_MONTH 10)
set(BUILD_DAY 18)
set(BUILD_HOUR 15)
set(BUILD_MINUTE 49)
set(BUILD_SECOND 48)

set(BUILD_TIMESTAMP  2021 10 18 15 49 48)

# Information about the SDK used to generate this.
set(GEN_REVISION 5991)
set(GEN_HASH a91aa1ab5517146f714ee92f1d3bdf74d60d619f)

EXECUTE_PROCESS(COMMAND git rev-list HEAD
                OUTPUT_VARIABLE BUILD_VERSION
                RESULT_VARIABLE GIT_STATUS
                ERROR_QUIET)
set(GIT_STATUS 1)
IF(NOT GIT_STATUS)
    string(REPLACE "\n" ";" BUILD_VERSION ${BUILD_VERSION})
    list(GET BUILD_VERSION 0 BUILD_HASH)
    list(LENGTH BUILD_VERSION BUILD_VERSION)
ELSE()
    # Not running from git, use the SDK version file.
    FILE(READ ${EM718X_TOP}/version BUILD_VERSION)
    string(REPLACE "\n" ";" BUILD_VERSION ${BUILD_VERSION})
    list(GET BUILD_VERSION 0 BUILD_VERSION)

    # Not running from git, use the SDK hash file.
    FILE(READ ${EM718X_TOP}/hash BUILD_HASH)
    string(REPLACE "\n" ";" BUILD_HASH ${BUILD_HASH})
    list(GET BUILD_HASH 0 BUILD_HASH)
ENDIF()

if(NOT DIST_TYPE)
    set(DIST_TYPE 7189_di03_rtos_bhi260)
endif()

SET(SVN_REVISION ${BUILD_VERSION})
SET(REVISION ${BUILD_VERSION})

set(ROM_TYPE ROM)
set(RAM_TYPE KERNEL_RAM)
set(FLASH_TYPE KERNEL_FLASH)
set(FLASH_BL_TYPE BOOTLOADER_FLASH)
set(USER_RAM_TYPE USER_RAM)
set(USER_FLASH_TYPE USER_FLASH)

set(KERNEL_STAGE kernel)
set(USER_STAGE user)

set(SDK 1)

include(${CMAKE_CURRENT_LIST_DIR}/common/config.cmake)

set(LIBRARIES_USER )

if(NOT IN_UTILS)
    IF(NOT SDK)
        install(FILES config.cmake DESTINATION .)
    ENDIF()
ENDIF()

