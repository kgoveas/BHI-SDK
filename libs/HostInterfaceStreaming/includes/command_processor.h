////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/command_processor.h
///
/// @project    EM7189
///
/// @brief      Host command handler for bootloader
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

#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#if _ARCVER /* Only use for arc build */

#include <types.h>
#include <arc.h>

#if !defined(PACK)
#define PACK __attribute__((__packed__))
#endif

#include <post_mortem.h>
#include <OTPCommon.h>
#include "commands.h"


//********** COMMAND BLOCKS ***********

typedef struct PACK ALIGNED(4) _erase_flash_command_
{
    COMMAND_HEADER header;
    UInt32 starting_sector;
    UInt32 ending_sector;
} ERASE_FLASH_COMMAND;

typedef struct PACK ALIGNED(4) _write_flash_command_
{
    COMMAND_HEADER header;
    UInt32 starting_sector;
    UInt8  data[120];
} WRITE_FLASH_COMMAND;


#define TEST_SUPPORT_SLEEP_MODE_NONE    (0u)
#define TEST_SUPPORT_SLEEP_MODE_DSLP    (1u)
#define TEST_SUPPORT_SLEEP_MODE_SLP     (2u)
#define TEST_SUPPORT_SLEEP_MODE_LSLP    (3u)
typedef struct PACK ALIGNED(4) _test_support_ags_
{
    UInt32 tosc_mode:1;     /**< @brief When set, timosc is enabled. Otherwise tosc is disabled. */
    UInt32 ram_en:4;        /**< @brief total number of ram banks to enable. */
    UInt32 sleep_test:2;    /**< @brief If non-zero, the part enters the requested sleep mode immediately. */
    UInt32 set_run_lvl:1;   /**< @brief If set, change to the requested run_lvl. */
    UInt32 run_lvl:2;       /**< @brief The requested run level. */
} test_support_ags_t;

typedef struct PACK ALIGNED(4) _test_support_command_
{
    COMMAND_HEADER header;
    test_support_ags_t test_argument;
} TEST_SUPPORT_COMMAND;


typedef struct PACK ALIGNED(4) _debug_test_command_
{
    COMMAND_HEADER header;
    UInt32 test_length;                                              // number of bytes to output, for tests that involve data transfer
    UInt8 test_num;
    UInt8 test_p0;
    UInt8 test_p1;
    UInt8 test_p2;
} DEBUG_TEST_COMMAND;

#define DT_PRAM 0
#define DT_FIFO 1

typedef struct PACK ALIGNED(4) _configure_sensor_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_id;
    float sample_rate_hz;       // customer may decide to use fixed point, e.g., 16.8, 12.12 etc.
    UInt16 latency_lsw_ms;
    UInt8 latency_msb_ms;       // allow up to 4.66 hours; need customer approval on range
} CONFIGURE_SENSOR_COMMAND;

typedef struct PACK ALIGNED(4) _change_sensor_dynamic_range_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_id;
    UInt16 dynamic_range;
    UInt8 reserved;
} CHANGE_SENSOR_DYNAMIC_RANGE_COMMAND;

typedef struct PACK ALIGNED(4) _dut_start_test_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_type;
    UInt8 pwr_modes;
    UInt8 test_mode; // step, start
    UInt8 low_num;
    UInt8 high_num;
    UInt8 reserved[3];
} DUT_START_TEST_COMMAND;

typedef struct PACK ALIGNED(4) _dut_get_group_info_
{
    COMMAND_HEADER header;
    UInt8 group;
    UInt8 reserved[3];
} DUT_GET_GROUP_INFO_COMMAND;

typedef struct PACK ALIGNED(4) _dut_get_test_info_
{
    COMMAND_HEADER header;
    UInt8 group;
    UInt8 test;
    UInt8 reserved[2];
} DUT_GET_TEST_INFO_COMMAND;


typedef struct PACK ALIGNED(4) _trigger_fatal_error_command_
{
    COMMAND_HEADER header;
    UInt8 error_type;
    UInt8 flags;
    UInt8 reserved[2];
} TRIGGER_FATAL_ERROR_COMMAND;

typedef struct PACK ALIGNED(4) _fifo_flush_command_
{
    COMMAND_HEADER header;
    UInt8 flush_value;
    UInt8 reserved[3];
} FIFO_FLUSH_COMMAND;

typedef struct PACK ALIGNED(4) _request_self_test_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_type;
    UInt8 reserved[3];
} REQUEST_SELF_TEST_COMMAND;

typedef struct PACK ALIGNED(4) _request_sensor_foc_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_type;
    UInt8 reserved[3];
} REQUEST_SENSOR_FOC_COMMAND;

typedef struct PACK ALIGNED(4) _control_bsx4_logging_command_
{
    COMMAND_HEADER header;
    UInt8 logging_mode;
    UInt8 reserved[3];
} CONTROL_BSX4_LOGGING_COMMAND;

typedef struct PACK ALIGNED(4) _set_sensor_foc_command_
{
    COMMAND_HEADER header;
    UInt8 sensor_id;
    UInt8 code;
    SInt16 x_offset;
    SInt16 y_offset;
    SInt16 z_offset;
} SET_SENSOR_FOC_COMMAND;

typedef struct PACK ALIGNED(4) _write_param_command_
{
    COMMAND_HEADER header;
    UInt8 data[124];
} WRITE_PARAM_COMMAND;

typedef struct PACK ALIGNED(4) _control_fifo_format_command_
{
    COMMAND_HEADER header;
    UInt8 flags;
    UInt8 reserved[3];
} CONTROL_FIFO_FORMAT_COMMAND;

#define CTRL_FIFO_DISABLE_TIMESTAMPS 0x01
#define CTRL_FIFO_DISABLE_HDR_TIMESTAMP 0x02

typedef struct PACK ALIGNED(4) _sdi_set_mode_command_
{
    COMMAND_HEADER header;
    UInt8 mode;
    UInt8 reserved[3];
} SDI_SET_MODE_COMMAND;

#define SDI_MODE_NORMAL    0x00
#define SDI_MODE_REAL_TIME 0x01
#define SDI_MODE_STEP      0x02

typedef struct PACK ALIGNED(4) _sdi_inject_data_command_
{
    COMMAND_HEADER header;
    UInt8 data[124];
} SDI_INJECT_DATA_COMMAND;


//********** STATUS BLOCKS ***********

typedef struct PACK ALIGNED(4) _debug_output_status_ // NOTE: only used by bootloader for debug test -- not for printf
{
    STATUS_HEADER header;
    UInt32 debug_format;                                             // 0 = ASCII, 1 = binary
} DEBUG_OUTPUT_STATUS;

#if ASYNC_STATUS_BLOCKS_ALLOWED
typedef struct PACK ALIGNED(4) _system_error_status_
{
    STATUS_HEADER header;
    UInt8 error;
    UInt8 interrupt_state;
    UInt8 debug_value;
    UInt8 debug_state;
    UInt32 sys_time;
    // followed by variable length context data...
} SYSTEM_ERROR_STATUS;
#endif

typedef struct PACK ALIGNED(4) _host_ev_timestamp_status_
{
    STATUS_HEADER header;
    UInt32 timestamp;
    UInt8 timestamp_msb;
    UInt8 reserved[3];
} HOST_EV_TIMESTAMP_STATUS;

typedef struct PACK ALIGNED(4) _sensor_self_test_results_
{
    STATUS_HEADER header;
    UInt8 sensor_id;
    UInt8 test_status;
    SInt16 x_offset;
    SInt16 y_offset;
    SInt16 z_offset;
} SENSOR_SELFTEST_RESULTS;

typedef enum _self_test_status_
{
    SELF_TEST_PASSED,
    SELF_TEST_X_FAILED,
    SELF_TEST_Y_FAILED,
    SELF_TEST_X_Y_FAILED,
    SELF_TEST_Z_FAILED,
    SELF_TEST_X_Z_FAILED,
    SELF_TEST_Y_Z_FAILED,
    SELF_TEST_ALL_FAILED,
    SELF_TEST_UNSUPPORTED,
    SELF_TEST_NO_DEVICE,
    SELF_TEST_DEVICE_BUSY,
} SELF_TEST_VALUES;

typedef struct PACK ALIGNED(4) _sensor_foc_results_
{
    STATUS_HEADER header;
    UInt8 sensor_id;
    UInt8 foc_status;
    SInt16 x_offset;
    SInt16 y_offset;
    SInt16 z_offset;
} SENSOR_FOC_RESULTS;

typedef struct PACK ALIGNED(4) _dut_status_
{
    STATUS_HEADER header;
    UInt8 status;
    UInt8 test_hi;
    UInt8 test_lo;
    UInt8 data[6]; // previously used AUX, AUX1, AUx2, AUX3, AUX4
    char  str[151];
} DUT_STATUS;

typedef struct PACK ALIGNED(4) _dut_num_groups_
{
    STATUS_HEADER header;
    UInt8 num_groups;
    UInt8 reserved[3];
} DUT_NUM_GROUPS;

typedef struct PACK ALIGNED(4) _dut_group_info_
{
    STATUS_HEADER header;
    UInt8 group;
    UInt8 num_tests;
    char  name[26];
} DUT_GROUP_INFO;

typedef struct PACK ALIGNED(4) _dut_test_info_
{
    STATUS_HEADER header;

    UInt8 group;
    UInt8 test;
    UInt8 pre_func;
    UInt8 test_func;

    UInt8 post_func;
    char  name[23];
} DUT_TEST_INFO;


typedef struct PACK ALIGNED(4) _sdi_status_
{
    STATUS_HEADER header;
    float sample_rate;
    UInt8 sensor_id;
    UInt8 reserved[3];
} SDI_STATUS;

typedef struct PACK ALIGNED(4) _post_mortem_report
{
    STATUS_HEADER hdr;
    post_mortem_info info;
    //UInt8 stack[0];
} post_mortem_report;

typedef struct PACK ALIGNED(4) _otp_contents_status_
{
    STATUS_HEADER header;
    UInt8 otp_data[OTP_DATA_BYTES + OTP_RESERVED_BYTES_DI02];
} OTP_CONTENTS_STATUS;



/**
 * \brief set when main command handler should be called
 */
extern volatile bool gCommandReady;
extern volatile bool gCommandPending;
extern volatile bool gStatusBlockPending;
extern volatile bool gProgramRAMUploadStarted;
extern volatile bool gProgramRAMUploadComplete;
extern volatile bool gProgramRAMStoresFlashData;
extern UInt32 status_block[64];

/**
 * \brief set up the command processor
 * \param running_main - set if we're in the main firmware now
 * \param buffer - where to store commands
 * \param buffer_size - how big it is in 32 bit words
 */
extern void JLI init_command_processor(bool running_main, UInt32 *buffer, UInt16 buffer_size);

/**
 * \brief this is called as soon as we have received the first DMA block of 128 bytes, or if a smaller amount is received
 * before the host terminates the transfer; it is called from the IRQ so must be fast
 */
extern void process_bootloader_command_start(void);

/**
 * \brief this is called when the upload channel has finished a host transfer; it is called from the IRQ so must be fast
 */
extern void process_bootloader_command_end(void);

/**
 * \brief this is called from the main bootloader task or loop; lengthy processing can be done here
 */
extern void process_bootloader_command_main(void);

/**
 * \brief this is called as soon as we have received at least a command header
 * \param length -
 * \param used_length -
 * \return bool -
 */
extern bool process_command(UInt32 length, UInt32 *used_length);


/**
 * \brief this is the main loop that handles commands (until OpenRTOS officially available)
 */
//lint -function(exit,bootloader_main_loop) /* Function Never returns */
extern void __attribute__((noreturn)) bootloader_main_loop(void);

/**
 * \brief this is called once the host has read a status block
 */
extern void process_status_end(void);

/**
 * \brief this is called once the host has read a FIFO block
 * \param ch - channel (WAKE or NONWAKE) which has been read
 */
extern void deassert_FIFO_int(UInt8 ch);

/**
 * \brief this is called when a data channel (1 or 2) finishes
 * \param gActiveChannel - channel number
 * \param total_xfer - number of bytes read
 */
typedef void (* HIF_FIFO_END_CALLBACK)(UInt8 gActiveChannel, UInt32 total_xfer);

/**
 * \brief place for main host interface (in program RAM or Flash) to attempt to handle incoming commands; used to extend the bootloader command set
 * \param len_available - number of bytes available (call get_upload_bytes to get them)
 * \param len_used - number actually used
 * \return bool - true if the callback was able to execute the command (had enough bytes of it)
 */
typedef bool (* HIF_COMMAND_CALLBACK)(UInt32 len_available, UInt32 *len_used);

/**
 * \brief if defined, intercept queue_debug() calls
 * \param sync - this is a synchronous response to a command, or an asynchronous message
 * \param hdr_length - length of the status block header
 * \param status - the status block header
 * \param data_length - length of the data array
 * \param data - optional bulk data to send (often from a different area of memory than status)
 * \return UInt16 - number of bytes queued or 0 if not handled
 */
typedef UInt16 (* HIF_STATUS_CALLBACK)(bool sync, UInt16 hdr_length, const STATUS_HEADER *status, UInt16 data_length, const UInt32 *data);

extern HIF_COMMAND_CALLBACK gHIFCommandCallback;
extern HIF_STATUS_CALLBACK gHIFStatusCallback;
extern HIF_FIFO_END_CALLBACK gHIFFIFOEndCallback;

#endif /* _ARCVER */
#endif /* !HOST_CHANNELS_H */
