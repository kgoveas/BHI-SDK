////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/commands.h
///
/// @project    EM7189
///
/// @brief      Host command and status definitions
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2017-2018 EM Microelectronic
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

#ifndef COMMANDS_H
#define COMMANDS_H

#if _ARCVER /* Only use for arc build */

#include <types.h>
#include <arc.h>

#if !defined(PACK)
#define PACK __attribute__((__packed__)) ALIGNED(4)
#endif

// passed to hook_HIFCommand()
#define CPH_START 0u
#define CPH_END   1u
#define CPH_MAIN  2u

// standard command header, according to the fw spec
typedef struct _command_header_
{
    UInt16 command;
    UInt16 length;
} COMMAND_HEADER;

// command codes for the bootloader
typedef enum _command_values_
{
    CMD_IDLE,                          // 0x0000
    CMD_DOWNLOAD_POST_MORTEM_DATA,     // 0x0001
    CMD_UPLOAD_TO_PROGRAM_RAM,         // 0x0002
    CMD_BOOT_PROGRAM_RAM,              // 0x0003
    CMD_ERASE_FLASH,                   // 0x0004
    CMD_WRITE_FLASH,                   // 0x0005
    CMD_BOOT_FLASH,                    // 0x0006
    CMD_SET_SENSOR_DATA_INJECTION_MODE,// 0x0007
    CMD_INJECT_SENSOR_DATA,            // 0x0008
    CMD_FIFO_FLUSH,                    // 0x0009
    CMD_SOFT_PASS_THROUGH,             // 0x000A
    CMD_REQUEST_SENSOR_SELF_TEST,      // 0x000B
    CMD_REQUEST_SENSOR_FOC,            // 0x000C
    CMD_CONFIGURE_SENSOR,              // 0x000D
    CMD_CHANGE_SENSOR_DYNAMIC_RANGE,   // 0x000E
    CMD_SET_CHANGE_SENSITIVITY,        // 0x000F
    CMD_DEBUG_TEST,                    // 0x0010
    CMD_DUT_CONTINUE,                  // 0x0011
    CMD_DUT_START_TEST,                // 0x0012
    CMD_TRIGGER_FATAL_ERROR,           // 0x0013
    CMD_CONTROL_BSX_LOGGING,           // 0x0014
    CMD_CONTROL_FIFO_FORMAT,           // 0x0015
    CMD_READ_OTP,                      // 0x0016
    CMD_TEST_SUPPORT,                  // 0x0017
    CMD_DUT_GET_NUM_GROUPS,            // 0x0018
    CMD_DUT_GET_GROUP_INFO,            // 0x0019
    CMD_DUT_GET_TEST_INFO,             // 0x001A

    CMD_LAST_MAIN,                     // last defined main firmware command (outside of the parameters)

    CMD_NORMAL_TOP = 0x00FF,           // 0x00FF -- end of normal command range
    CMD_SET_PARAMETER_BASE = 0x0100,   // 0x0100 -- upper nibble = 0; least significant 12 bits are parameter page and parameter
    CMD_SET_PARAMETER_TOP = 0x0FFF,    // 0x0FFF
    CMD_GET_PARAMETER_SEL = 0x1000,    // 0x1000 -- bit set to indicate get vs. set
    CMD_GET_PARAMETER_BASE = 0x1100,   // 0x1100 -- upper nibble = 1; 12 remaining bits are parameter page and parameter
    CMD_GET_PARAMETER_TOP = 0x1FFF,    // 0x1FFF
} COMMAND_CODE;


typedef struct PACK ALIGNED(4) _status_header_
{
    UInt16 status_code;
    UInt16 length;
} STATUS_HEADER;

typedef enum _status_values_
{
    STAT_IDLE,                        // 0x00
    STAT_INITIALIZED,                 // 0x01
    STAT_RESERVED1,                   // 0x02 unused
    STAT_CRASH_DUMP,                  // 0x03
    STAT_INJECTED_SENSOR_CONFIG_REQ,  // 0x04
    STAT_PASSTHROUGH_RESULTS,         // 0x05
    STAT_SENSOR_SELFTEST_RESULTS,     // 0x06
    STAT_SENSOR_FOC_RESULTS,          // 0x07
    STAT_RESERVED2,                   // 0x08 unused
    STAT_RESERVED3,                   // 0x09 unused
    STAT_FLASH_ERASE_COMPLETE,        // 0x0A
    STAT_FLASH_WRITE_COMPLETE,        // 0x0B
    STAT_RESERVED4,                   // 0x0C unused
    STAT_HOST_EV_TIMESTAMP,           // 0x0D
    STAT_DUT_TEST,                    // 0x0E
    STAT_COMMAND_ERROR,               // 0x0F
    STAT_OTP_CONTENTS,                // 0x10
    STAT_DUT_GROUP_INFO,              // 0x11
    STAT_DUT_NUM_GROUPS,              // 0x12
    STAT_DUT_TEST_INFO,               // 0x13
    STAT_GET_PARAMETER_BASE = 0x0100, // 0x0100 -- upper nibble = 0; least significant 12 bits are parameter page and parameter
    STAT_GET_PARAMETER_TOP = 0x0FFF,  // 0x0FFF
} STATUS_CODE;


typedef struct PACK ALIGNED(4) _command_error_status_
{
    STATUS_HEADER header;
    UInt16 command;
    UInt8  error;
    UInt8  reserved;
} COMMAND_ERROR_STATUS;

#define CMD_ERR_STAT_SUCCESS            (0x00u)     /* The command completed successfully */

#define CMD_ERR_STAT_INCORRECT_LENGTH   (0x01u)     /* An incorrect length / or number of parameters was specified. */
#define CMD_ERR_STAT_TOO_LONG           (0x02u)     /* The command has a valid length, however the command processor buffer is not large enough to hold it. */
#define CMD_ERR_STAT_PARAM_WRITE        (0x03u)     /* The command (param write) did not complete successfully */
#define CMD_ERR_STAT_PARAM_READ         (0x04u)     /* The command (param read) did not complete successfully */
#define CMD_ERR_STAT_INVALID_COMMAND    (0x05u)     /* The specified command does not exist */
#define CMD_ERR_STAT_INVALID_PARAMETER  (0x06u)     /* A command parameter is invalid */
#define CMD_ERR_STAT_COMMAND_FAILED     (0xFFu)     /* The command did not complete successfully */


/**
 * \brief start transferring a status block to the host
 * \param sync - this is in response to a command; otherwise, is any async notification
 * \param status - the structure to transfer; the length field + sizeof(STATUS_HEADER) determines the DMA transfer size
 */
extern void JLI queue_status(bool sync, const STATUS_HEADER *status);

/**
 * \brief start transfering a large status block that includes bulk data
 * \param sync - this is in response to a command; otherwise, is any async notification
 * \param hdr_length - number of bytes pointed to by the status pointer to send;
 * status->length - hdr_length will be the number to send from the data pointer
 * \param status - the starting structure
 * \param data_length - the length of the data
 * \param data - a pointer to the bulk data
 * \return UInt16 - number of bytes actually stored
 */
extern UInt16 JLI queue_debug(bool sync, UInt16 hdr_length, const STATUS_HEADER *status, UInt16 data_length, UInt32 *data);

/**
 * \brief
 * \param command -
 * \param error -
 * \return void JLI -
 */
void JLI report_command_error(UInt16 command, UInt8 error);

/**
 * \brief
 * \param callback -
 * \return HIF_CUSTOM_CMD_CALLBACK JLI -
 */
typedef bool (* HIF_CUSTOM_CMD_CALLBACK)(const COMMAND_HEADER *hdr, UInt32 len_available, UInt32 *len_used, UInt8 phase);
extern HIF_CUSTOM_CMD_CALLBACK JLI registerHIFCustomCmdHandler(HIF_CUSTOM_CMD_CALLBACK callback);

extern HIF_CUSTOM_CMD_CALLBACK gHIFCustomCmdCallback;

#endif /* _ARCVER */
#endif /* !COMMANDS_H */
