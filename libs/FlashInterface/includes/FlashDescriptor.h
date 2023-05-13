////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/FlashInterface/includes/FlashDescriptor.h
///
/// @project    EM7189
///
/// @brief      QSPI Flash driver.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016-2018 EM Microelectronic
/// @cond
///
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// 1. Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// 2. Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
///
////////////////////////////////////////////////////////////////////////////////
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @endcond
////////////////////////////////////////////////////////////////////////////////

/** @defgroup FLASH_INT    Flash Interface */
/** @addtogroup FLASH_INT
 * @{
 */
#ifndef FLASH_DESCRIPTOR_H
#define FLASH_DESCRIPTOR_H

#include <arc.h>

#define FLASH_SIGNATURE     (0xf1a5f1a7)    /**< @brief The flash signature signifying a valid flash descriptor. */
#define FLASH_EXT_SIGNATURE (0xfe1eae5e)

#define FLASH_SIZE_1MB      0x00    /**< @brief The flash device is 1MB in size */
#define FLASH_SIZE_2MB      0x01    /**< @brief The flash device is 2MB in size */
#define FLASH_SIZE_4MB      0x02    /**< @brief The flash device is 4MB in size */
#define FLASH_SIZE_8MB      0x03    /**< @brief The flash device is 8MB in size */

typedef struct {
    /**
     * @brief The control0 register contents to define a command.
     */
    RegQSPIControl0_t       control0;
    /**
     * @brief The control1 register contents to define a command.
     */
    RegQSPIControl1_t       control1;

    /**
     * @brief The instruction0 register contents to define a command.
     */
    RegQSPIInstruction0_t   cmd;

    /**
     * @brief The instruction1 register contents to define a command.
     */
    RegQSPIInstruction1_t   cfg;
} FlashCommand_t;

typedef struct {
    /** @brief  Full Chip Erase Command */
    FlashCommand_t      chipErase;

    /** @brief  Sector Erase Command (usually 4K)*/
    FlashCommand_t      sectorErase;

    /** @brief Read Command */
    FlashCommand_t      read;

    /** @brief Write Command */
    FlashCommand_t      write;

    // Previous ROM versions did not support generic read/write commands in the descriptor.
    /** @brief The initialization index in the initSequence array for configuring the flash device. */
    UInt16              initSequenceIndex;

    /** @brief The number of commands to run in the initSequence array for configuring the flash device. */
    UInt16              initSequenceLength;

    /** @brief The initialization index at which point to switch over to the new descriptor. Switchover completes after the command has executed. Used to ensure that the QSPI_Busy function uses the correct mode. */
    UInt16              initSwitchoverIndex;

    UInt16              rsvd;
} FlashCommandSet_t;

typedef struct  {
    /** @brief Generic Read Command */
    FlashCommand_t      readCmd;

    /** @brief Generic Write Command */
    FlashCommand_t      writeCmd;
} FlashExtendedCommandSet_t;

typedef struct {
    UInt32 signature;
    /** @brief The extended command set when running in Long Run mode. */
    FlashExtendedCommandSet_t   longRun;
    /** @brief The extended command set when running in Turbo mode. */
    FlashExtendedCommandSet_t   turbo;
} FlashDescriptorExtension_t;


typedef struct {
    /** @brief The flash signature, set to @ref FLASH_SIGNATURE when valid. */
    UInt32 signature;

    /** @brief The flash size. Valid values are @ref
     *         FLASH_SIZE_1MB, @ref FLASH_SIZE_2MB, @ref
     *         FLASH_SIZE_4MB, @ref FLASH_SIZE_8MB.  For 500KB
     *         Flash specify @ref FLASH_SIZE_1MB and set
     *         flash_size_512K flag */
    UInt8       size;

    /** @brief The number of bytes erased (and it's alignment) when a sector erase command is issued */
    UInt16      eraseSize;  // Erase sector size
    /** @brief The maximum number of bytes that can be written at in a single transaction */
    UInt16      writeSize;  // Write Page size.

    /** @brief The command set when running in Long Run mode. */
    FlashCommandSet_t   clockLongRun;

    /** @brief The command set when running in Turbo mode. */
    FlashCommandSet_t   clockTurbo;

    /** @brief The initialization sequence to place the flash device in a known state after startup. */
    struct {
        FlashCommand_t      command;
        UInt8               payload[16];
    }   initSequence[18];

    UInt8       qspi_fast:1;    /**< @brief Tell hw to run a fast QSPI speed instead of SYSOSC/2 */
    UInt8       turbo_en:1;     /**< @brief If set, allows the bootloader to enter turbo mode during verification. */
    UInt8       no_boot_after_err:1; /**< @brief If set will prevent the Flash firmware from autobooting following a fatal error. */
    UInt8       flash_size_512K:1;
    UInt8       reserved:4;

    FlashDescriptorExtension_t   extended;
} FlashDescriptor_t;


/** @cond */
_Static_assert(sizeof(FlashDescriptor_t) <= 4096, "FlashDescriptor_t must be 4096 bytes or less");
/** @endcond */

#endif /* FLASH_DESCRIPTOR_H */

/** @} */
