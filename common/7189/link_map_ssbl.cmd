################################################################################
##
##
## Project       : 718x
##
## Description   : Linker script common map file
##
################################################################################
##
################################################################################
##                          Copyright
################################################################################
##
##         Copyright (C) EM Microelectronic US Inc.
##
## Disclosure to third parties or reproduction in any form what-
## soever, without prior written consent, is strictly forbidden
##
################################################################################

CRC_SIZE = 4;
ROM_SIZE = 144K - CRC_SIZE;
PERIPH_SIZE = 1M;

INITIAL_BANK_SIZE  = 16K;
OPTIONAL_BANK_SIZE = 32K;
_OPTIONAL_RAM_BANKS = OPTIONAL_RAM_BANKS;

MAX_CODE_RAM_SIZE = INITIAL_BANK_SIZE + (OPTIONAL_RAM_BANKS * OPTIONAL_BANK_SIZE);
MAX_DATA_RAM_SIZE = INITIAL_BANK_SIZE + (OPTIONAL_RAM_BANKS * OPTIONAL_BANK_SIZE);

MAX_FLASH_FW_SIZE = MAX_FLASH_FW_SIZE_SECTORS_LN * 4 * 1K;
TOTAL_FLASH_SIZE  = 8192K;
FLASH_DESCR_SIZE  = 4K;
FLASH_HDR_SIZE    = 4K;
FLASH_BOOT_FW_SIZE = 16K;
FLASH_BOOT_TOTAL_SIZE = FLASH_HDR_SIZE + FLASH_BOOT_FW_SIZE;
FLASH_FW_SIZE     = FLASH_HDR_SIZE + MAX_FLASH_FW_SIZE;
FLASH_REM_SIZE    = TOTAL_FLASH_SIZE - FLASH_DESCR_SIZE - FLASH_BOOT_TOTAL_SIZE - (2 * FLASH_FW_SIZE);

MEMORY {
    ROM:               ORIGIN = 0x0010_0000 LENGTH = ROM_SIZE
    ROM_CRC:           ORIGIN = 0x0012_3FFC LENGTH = CRC_SIZE

    FW_HEADER:         ORIGIN = 0x0012_3F84 LENGTH = 124
    CODE_RAM:          ORIGIN = 0x0012_4000 LENGTH = MAX_CODE_RAM_SIZE
    DATA_RAM:          ORIGIN = 0x00A0_4000 LENGTH = MAX_DATA_RAM_SIZE

    FLASH_DESCR:       ORIGIN = 0x0020_0000 LENGTH = FLASH_DESCR_SIZE
    FLASH_BOOT_HDR:    ORIGIN = 0x0020_1F84 LENGTH = 124
    FLASH_BOOT_FW:     ORIGIN = 0x0020_2000 LENGTH = FLASH_BOOT_FW_SIZE
    FLASH_PRIMARY_HDR: ORIGIN = 0x0020_6F84 LENGTH = 124
    FLASH_PRIMARY_FW:  ORIGIN = 0x0020_7000 LENGTH = MAX_FLASH_FW_SIZE
    FLASH_SHADOW_NEW:  ORIGIN = 0x0020_7000 + MAX_FLASH_FW_SIZE + 0x0000_0F80 LENGTH = 4
    FLASH_SHADOW_HDR:  ORIGIN = 0x0020_7000 + MAX_FLASH_FW_SIZE + 0x0000_0F84 LENGTH = 124
    FLASH_SHADOW_FW:   ORIGIN = 0x0020_8000 + MAX_FLASH_FW_SIZE               LENGTH = MAX_FLASH_FW_SIZE
    FLASH:             ORIGIN = 0x0020_8000 + MAX_FLASH_FW_SIZE * 2           LENGTH = FLASH_REM_SIZE

    CACHE_TAG:         ORIGIN = 0x00B0_0000 LENGTH = 1024

    PERIPHERALS:       ORIGIN = 0x00F0_0000 LENGTH = PERIPH_SIZE

    FPGA_OTP:          ORIGIN = 0x0000_0000 LENGTH = 128
}
