////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/FlashInterface/includes/QSPIInterface.h
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

/** @addtogroup FLASH_INT
 * @{
 */

#ifndef QSPI_INTERFACE_H
#define QSPI_INTERFACE_H


// Generic commands
#define READ_JEDEC_ID       (0x9Fu)     /**< @brief The standard QSPI command for reading out the manufacturer id. */

#define READ_STATUS1        (0x05u)     /**< @brief The standard QSPI command for reading out the status register. */
#define WRITE_STATUS1       (0x01u)     /**< @brief The standard QSPI command for writing the status register. */
#define READ_STATUS2        (0x35u)     /**< @brief The standard QSPI command for reading out the second status register. */
#define WRITE_STATUS2       (0x31u)     /**< @brief The standard QSPI command for writing the second status register. */


#define WRITE_ENABLE        (0x06u)     /**< @brief The standard QSPI commend for enabling writes on the device. */
//#define WRITE_ENABLE_STATUS
#define PAGE_PROGRAM        (0x02u)     /**< @brief The standard QSPI commend for writing a block of data to the device. */
#define READ_DATA           (0x03u)     /**< @brief The standard QSPI commend for reading data. */

#define CHIP_ERASE          (0xC7u)     /**< @brief The standard QSPI commend for erasing the complete device. */
#define SECTOR_ERASE        (0x20u)     /**< @brief The standard QSPI commend for erasing a sector on the device. */



#define STATUS1_BUSY    (0x01u)         /**< @brief The Busy bit position in the status register. */
#define STATUS1_WEL     (0x02u)         /**< @brief The Write Enable (WEL) bit position in the status register. */





// Part specific: Winbond
#define DISABLE_QPI         (0xFFu)     /**< @brief the Winbond specific command for exiting QPI mode and resuming SSPI mode */



/**
 * @cond Internal
 */
/////////////////////////////////////////////////////
// # | Command  | Addr   | Dummy | Data   |
// 0 |          |        |       |        |
// 1 | Single   | Single | 0     | Single |
// 2 | Single   | Single | Hi-Z  | Dual   |
// 3 | Single   | Single | Hi-Z  | Quad   |
// 4 | Single   | Dual   | Hi-Z  | Dual   |
// 5 | Single   | Quad   | Hi-Z  | Quad   |
// 6 | Dual     | Dual   | Hi-Z  | Dual   |
// 7 | Quad     | Quad   | Hi-Z  | Quad   |
#define DATA_SSPI           (0x01u)

#define DATA_DSPI           (0x00u)
#define DATA_QSPI           (0x01u)

#define ADDR_SSPI           (0x00u)
#define ADDR_SAME_DATA      (0x04u)

#define COMMAND_SSPI        (0x00u) // outside of bit range.
#define COMMAND_SAME_ADDR   (0x02u)


#define PROTOCOL1       (COMMAND_SSPI      | ADDR_SSPI      | DATA_SSPI) // 0 | 0 | 1   1 // NOTE: This is a special case.

#define PROTOCOL2       (COMMAND_SAME_ADDR | ADDR_SSPI      | DATA_DSPI) // 2 | 0 | 0   2
#define PROTOCOL3       (COMMAND_SAME_ADDR | ADDR_SSPI      | DATA_QSPI) // 2 | 0 | 1   3

#define PROTOCOL4       (COMMAND_SSPI      | ADDR_SAME_DATA | DATA_DSPI) // 0 | 4 | 0   4
#define PROTOCOL5       (COMMAND_SSPI      | ADDR_SAME_DATA | DATA_QSPI) // 0 | 4 | 1   5
#define PROTOCOL6       (COMMAND_SAME_ADDR | ADDR_SAME_DATA | DATA_DSPI) // 2 | 4 | 0   6
#define PROTOCOL7       (COMMAND_SAME_ADDR | ADDR_SAME_DATA | DATA_QSPI) // 2 | 4 | 1   7
/**
 * @endcond Internal
 */

typedef enum {
    QSPIProtoSSPI = PROTOCOL1, /**< @brief SSPI command/addr, SSPI data */
    QSPIProtoDSPI = PROTOCOL2, /**< @brief SSPI command/addr, DSPI data */
    QSPIProtoQSPI = PROTOCOL3, /**< @brief SSPI command/addr, QSPI data */
    // 4
    // 5
    QSPIProtoDPI  = PROTOCOL6, /**< @brief DSPI command/addr, DSPI data */
    QSPIProtoQPI  = PROTOCOL7, /**< @brief QSPI command/addr, QSPI data */
} QSPI_Proto_t;

/**
 * @fn void QSPI_enable(void);
 * @brief   This functions enables the QSPI interface.
 *
 * The QSPI interface must be enabled before any QSPI commands will complete.
 */
void QSPI_enable(void);

/**
 * @fn void QSPI_disable(void);
 * @brief   This functions disables the QSPI interface.
 *
 * The QSPI interface must be enabled before any QSPI commands will complete.
 */
void QSPI_disable(void);

/**
 * @fn bool QSPI_isEnabled(void);
 * @brief   This functions determines if the QSPI interace is enabled.
 *
 * @returns The current status of the QSPI interface.
 */
bool QSPI_isEnabled(void);

/**
 * @fn void QSPI_updatePulls(QSPI_Proto_t protocol);
 * @brief   This functions updates the MFPAD pulls into a state ideal for a specific protocol.
 *
 * @param protocol  The protocol to use for the MFPAD pull selection.
 */
void QSPI_updatePulls(QSPI_Proto_t protocol);


/**
 * @fn void QSPI_runFlashCommand(const FlashCommand_t* command, UInt32* rw_buffer);
 * @brief   This function causes a generic QSPI command to be send out the bus.
 *
 * @param [in] command          The specified QSPI command to execute.
 * @param [in,out] rw_buffer    The location to write data to (for read transactons) or to output on the bus (for write transactions).
 */
void QSPI_runFlashCommand(const FlashCommand_t* command, UInt32* rw_buffer);

/**
 * @fn void QSPI_cmd(UInt8 command);
 * @brief   This functions issues a single-byte command to the QSPI device with no payload.
 */
void QSPI_cmd(UInt8 command);

/**
 * @fn void QSPI_QuadCmd(UInt8 command);
 * @brief   This functions issues a single-byte command to the QSPI device with no payload, forcing the bus into QPI mode first.
 */
void QSPI_QuadCmd(UInt8 command);

#define QSPI_MAX_PAYLOAD    (256u)      /**< @brief The maximum number of bytes that can be send in a single transaction over the QSPI interface. */


/**
 * @fn void QSPI_writeCmd(UInt8 command, const UInt32* buffer, UInt16 size);
 * @brief   This functions issues a single-byte command to the QSPI device with a payload.
 *
 * @param command           The command to write out the QSPI bus.
 * @param [in] buffer       The buffer to write out the QSPI bus.
 * @param size              The number of bytes in the buffer to output on the bus.
 */
//lint -sem( QSPI_WriteCmd, 2p) // buffer cannot be null.
//lint -sem( QSPI_WriteCmd, 3n <= 2P && 3n <= 256) // size must be not underrun buffer, Must be <= 256
void QSPI_writeCmd(UInt8 command, const UInt32* buffer, UInt16 size);

/**
 * @fn void QSPI_readCmd(UInt8 command, UInt32* buffer, UInt16 size);
 * @brief   This functions issues a single-byte command to the QSPI device with read data.
 *
 * @param command           The command to write out the QSPI bus.
 * @param [out] buffer       The buffer to read data into from the QSPI bus.
 * @param size              The number of bytes to read.
 */
//lint -sem( QSPI_ReadCmd, 2p) // buffer cannot be null.
//lint -sem( QSPI_ReadCmd, 3n <= 2P && 3n <= 256) // size must be not underrun buffer, Must be <= 256
void QSPI_readCmd(UInt8 command, UInt32* buffer, UInt16 size);

/**
 * @fn UInt32 QSPI_readCmdFast(UInt8 command, UInt16 size);
 * @brief   This functions issues a single-byte command to the QSPI device with read data.
 *
 * @param command           The command to write out the QSPI bus.
 * @param size              The number of bytes to read, up to 4.
 *
 * @returns The bytes read on the bus.
 */
//lint -sem( QSPI_ReadCmdFast, 2n <= 4) // size must be 4 bytes or less
UInt32 QSPI_readCmdFast(UInt8 command, UInt16 size);

/**
 * @fn void QSPI_writeData(UInt32 addr, const UInt32* buffer, UInt16 size);
 * @brief   This functions writes data to the specified address in the QSPI device..
 *
 * @param addr              The address to write the data.
 * @param [in] buffer       The data to write.
 * @param size              The number of bytes to write.
 *
 * Note: this does not erase the addresses being written to first.
 */
//lint -sem( QSPI_WriteData, 2p) // buffer cannot be null.
//lint -sem( QSPI_WriteData, 3n <= 2P && 3n <= 256) // size must be not underrun buffer, Must be <= 256
void QSPI_writeData(UInt32 addr, const UInt32* buffer, UInt16 size);

/**
 * @fn void QSPI_readData(UInt32 addr, UInt32* buffer, UInt16 size);
 * @brief   This functions reads data from the specified address.
 *
 * @param addr              The address to read.
 * @param [out] buffer      The location to store the read in data.
 * @param size              The number of bytes to read.
 *
 * Note: this does not erase the addresses being written to first.
 */
//lint -sem( QSPI_ReadData, 2p) // buffer cannot be null.
//lint -sem( QSPI_ReadData, 3n <= 2P && 3n <= 256) // size must be not underrun buffer, Must be <= 256
void QSPI_readData(UInt32 addr, UInt32* buffer, UInt16 size);

/**
 * @fn void QSPI_chipErase(void);
 * @brief   This functions issues the full-chip erase command on the attached QSPI device.
 *
 * NOTE: Most flash devices will reject this command if any sectors on the device are locked.
 */
void QSPI_chipErase(void);

/**
 * @fn void QSPI_sectorErase(UInt32 addr);
 * @brief   This functions issues the sector erase command on the attached QSPI device.
 *
 * NOTE: The address provided will be aligned to the eraseSize by the flash device.
 *       Care should be taken to ensure that additional data than desired is not erased.
 */
void QSPI_sectorErase(UInt32 addr);

/**
 * @fn static int ALWAYS_INLINE QSPI_isBusy(void);
 * @brief This function asks the attached QSPI device if it is free to accept additional command.
 *
 * In most cases, the QSPI device will become busy after any write or erase command.
 */
static int ALWAYS_INLINE QSPI_isBusy(void)
{
    UInt32 status = QSPI_readCmdFast(READ_STATUS1, 1);

    return status & STATUS1_BUSY;
}


/**
 * @fn static int ALWAYS_INLINE QSPI_isWriteEnabled(void);
 * @brief This function asks the attached QSPI device if writes are allowed.
 *
 * In most cases, writes become automatically dis-asslowed after any write or erase command.
 */
static int ALWAYS_INLINE QSPI_isWriteEnabled(void)
{
    UInt8 status = QSPI_readCmdFast(READ_STATUS1, 1);

    return status & STATUS1_WEL;
}

/**
 * @fn static void ALWAYS_INLINE QSPI_enableWrites(void);
 * @brief This function tells the attached QSPI device that writes are now allowed.
 *
 * In most cases, writes become automatically dis-asslowed after any write or erase command.
 */
static void ALWAYS_INLINE QSPI_enableWrites(void)
{
    QSPI_cmd(WRITE_ENABLE);
}

/**
 * @fn bool QSPI_FCCDisable(void);
 * @brief This disables the flash cache controller..
 *
 * The flash cache controller must be disabled before any flash related command will complete.
 * @returns The state of the cache controller before disabling it.
 */
bool QSPI_FCCDisable(void);

/**
 * @fn bool QSPI_FCCEnabled(void);
 * @brief Requests the state of the cache controller.
 *
 * @returns The state of the cache controller.
 */
static bool ALWAYS_INLINE QSPI_FCCEnabled(void)
{
    return (bool)FCC.Control.r32;
}


/**
 * @fn void QSPI_FCCRestore(bool fcc_en);
 * @brief This restores the state flash cache controller.
 *
 * The flash cache controller must be enabled before executing code out of flash.
 *
 * @param fcc_en    Wheather to restore the cache controller into the enabled or disabled state.
 */
void QSPI_FCCRestore(bool fcc_en);

/**
 * @fn void QSPI_puts(const char* string);
 * @brief This function outputs a string via the QSPI interface in the EM100Pro debugger format.
 *
 * @param [in] string   The string to output on the bus.
 */
void QSPI_puts(const char* string);

/**
 * @fn size_t QSPI_fwrite_time(const void *ptr, size_t elem, size_t count, void *fp, UInt8 packet_header);
 * @brief This function outputs generic data via the QSPI interface in the EM100Pro debugger format.
 *
 * @param [in] ptr      The data to output
 * @param elem          The size of each element to output.
 * @param count         The number of elements to output.
 * @param fp            Unused
 * @param packet_header Unused
 */
#include <stdio.h>
size_t QSPI_fwrite_time(const void *ptr, size_t elem, size_t count, void *fp, UInt8 packet_header);

#endif /* !QSPI_INTERFACE_H */

/** @} */
