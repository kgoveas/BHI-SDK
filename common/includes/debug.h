////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/debug.h
///
/// @project    EM7189
///
/// @brief      Debug routines.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

#ifndef _INC_DEBUG_H
#define _INC_DEBUG_H

#if defined(_ARCVER)
    #include <aux_registers.h>
#endif

#ifndef DEBUG_EN
#define DEBUG_EN  1 /**< Enable debugging by default, may be changed to 0 to disable */
#endif

// Error Register Values
#define SUCCESS               0x00     /**< @brief No errors have occurred */
#define INIT_PASS             0x01
#define TEST_PASS             0x02
#define FOC_PASS              0x03

/** Firmware upload error codes **/
#define ERROR_FWU_VERSION_MISMATCH          (0x10)  /**< @brief Expected version mismatch */
#define ERROR_FWU_HDR_CRC                   (0x11)  /**< @brief Calculated CRC of firmware header CRC does not header CRC in firmware header */
#define ERROR_FWU_SHA_MISMATCH              (0x12)  /**< @brief Calculated SHA of firmware image does not match SHA in firmware header */
#define ERROR_FWU_IMG_CRC                   (0x13)  /**< @brief Calculated CRC of firmware image does not match image CRC in firmware header */
#define ERROR_FWU_ECDSA                     (0x14)  /**< @brief ECDSA signature does not verify */
#define ERROR_FWU_PUB_KEY                   (0x15)  /**< @brief Public key not found at specified location or CRC error on key */
#define ERROR_FWU_UNSIGNED_RESTRICTED       (0x16)  /**< @brief Unsigned image not allowed when public key present in OTP */
#define ERROR_FWU_FW_HDR_NOT_PRESENT        (0x17)  /**< @brief No firmware header at this location or firmware header magic number mismatch */

/** Other init error codes */
#define ERROR_HARDWARE_FAILURE              (0x18)
#define ERROR_UNEXPECTED_WD_RESET           (0x19)  /**< @brief Unexpected watchdog reset. */
#define ERROR_ROM_VERSION_MISMATCH          (0x1A)  /**< @brief The rom version does not match the expected version. */
#define ERROR_FATAL_ERROR                   (0x1B)  /**< @brief A fatal error was triggered by the firmware. */
#define ERROR_PAYLOAD_NOT_FOUND             (0x1C)  /**< @brief The next payload was not included in the upload. */
#define ERROR_PAYLOAD_NOT_VALID             (0x1D)  /**< @brief The payload included is no executable. */
#define ERROR_PAYLOAD_ENTRIES_INVALID       (0x1E)  /**< @brief The payload does not have all valid entries specified. */
#define ERROR_OTP_CRC_INVALID               (0x1F)  /**< @brief The trim CRC programmed in OTP is incorrect. */
/** Sensor Specific Codes **/
#define ERROR_INIT_FAILED                   (0x20)  /**< @brief 718x has failed to initialize */
#define ERROR_UNEXPECTED_DEVICE             (0x21)  /**< @brief An unexpected device was detected */
#define ERROR_NO_DEVICE                     (0x22)  /**< @brief No device was found */
#define ERROR_UNKNOWN                       (0x23)  /**< @brief An unknown error occurred */
#define ERROR_DATA_UNAVAILABLE              (0x24)  /**< @brief The device is not providing valid data */
#define ERROR_SLOW_SAMPLE_RATE              (0x25)  /**< @brief The device is not providing data at the requested rate */
#define ERROR_DATA_OVERFLOW                 (0x26)  /**< @brief The device data is out of range */
#define ERROR_STACK_OVERFLOW                (0x27)  /**< @brief The system stack has overflowed and needs to be increased */
#define ERROR_INSUFFICENT_FREE_RAM          (0x28)  /**< @brief Not enough program and data RAM provided to allocate FIFOs */
#define ERROR_SNS_DRV_PARSE_FAILED          (0x29)  /**< @brief Parsing of sensor drivers during init failed */
#define ERROR_RAM_BANKS_INVALID             (0x2A)  /**< @brief Programming error led to invalid RAM bank configuration */
#define ERROR_INVALID_EVENT_SPECIFIED       (0x2B)  /**< @brief Event value out of range when trying to register event handler */
#define ERROR_MORE_THAN_32_ON_CHANGE        (0x2C)  /**< @brief Only the first 32 on-change sensors are supported by lost-on-change recovery code */
#define ERROR_FIRMWARE_TOO_LARGE            (0x2D)  /**< @brief The firmware is too large (or allocated too much dccm statically), and as a result does not have enough ram banks to meet the ram requirements. */
#define ERROR_CONFIG_NOT_PRESENT            (0x2E)  /**< @brief The firmware global configuration structure was not provided. */
#define ERROR_INVALID_BANKS                 (0x2F)  /**< @brief The firmware determined that an invalid number of banks needed to be turned on. */

#define ERROR_MW_MATH                       (0x30)  /**< @brief The double precision math library triggered an error */

// NOTE: The interrupt error codes are hard coded in vectors.s. If these values change, vectors.s must also be updated.
#define ERROR_MEM_ERROR                     (0x40)  /**< @brief A memory error occurred */
#define ERROR_SWI3                          (0x41)  /**< @brief SWI3 was triggered without an interrupt handler */
#define ERROR_SWI4                          (0x42)  /**< @brief SWI4 was triggered without an interrupt handler */
#define ERROR_INST_ERROR                    (0x43)  /**< @brief An illegal instruction was executed */
#define ERROR_UNHANDLED_INT                 (0x44)
#define ERROR_INVALID_MEMORY_ACCESS         (0x45)  /**< @brief One of the CHP.Diagnostic error bits was set; check interrupt state and debug value for copy of this register */

#define ERROR_ALGO                          (0x50)  /**< @brief An algorithm specific error has been detected */


#define ERROR_SELF_TEST_FAIL                (0x60)  /**< @brief The sensor self test has failed. */
#define ERROR_SELF_TEST_X_FAIL              (0x61)  /**< @brief The X axis on the sensor has failed. */
#define ERROR_SELF_TEST_Y_FAIL              (0x62)  /**< @brief The Y axis on the sensor has failed. */
#define ERROR_SELF_TEST_Z_FAIL              (0x64)  /**< @brief The Z axis on the sensor has failed. */

#define ERROR_FOC_FAIL                      (0x65)  /**< @brief The FOC on the sensor has failed. */

#define ERROR_SENSOR_BUSY                   (0x66)  /**< @brief cannot perform self-test or FOC because sensor is actively being used */
#define ERROR_TEST_UNSUPPORTED              (0x6F)  /**< @brief The sensor does not support the requested mode. */


#define ERROR_NO_PROGRAM_RAM_LEFT           (0x70)  /**< @brief we failed to allocate room for our data structures in program RAM */
#define ERROR_CODE_AND_DATA_NOT_ADJACENT    (0x71)  /**< @brief we expected the code RAM and data RAM to be adjacent in address space, but they are not */
#define ERROR_NO_HOST_INTERRUPT             (0x72)  /**< @brief the host interrupt value did not have the proper bits set */
#define ERROR_HOST_EVENT_NO_SIZE            (0x73)  /**< @brief the event id passed to the host interface has no known size. */
#define ERROR_NO_DATA_RAM_LEFT              (0x74)  /**< @brief data RAM boundaries don't make sense; fatal error */
#define ERROR_HOST_CHANNEL_UNDERFLOW        (0x75)  /**< @brief DMA channel did not keep up with host */
#define ERROR_HOST_CHANNEL_OVERFLOW         (0x76)  /**< @brief host did not keep up with DMA channel */
#define ERROR_HOST_CHANNEL_EMPTY            (0x77)  /**< @brief host read channel when there was nothing left to read */
#define ERROR_DMA_ERROR                     (0x78)  /**< @brief uDMA engine error */
#define ERROR_CORRUPTED_IN_CHAIN            (0x79)  /**< @brief DMA chain links on input channel are not correct */
#define ERROR_CORRUPTED_OUT_CHAIN           (0x7A)  /**< @brief DMA chain links on output channel are not correct */
#define ERROR_BUF_MGMT                      (0x7B)  /**< @brief problem with buffer management */
#define ERROR_IN_CH_NOT_WORD_ALIGNED        (0x7C)  /**< @brief command input channel has 1, 2, or 3 bytes; abort_command() performed to recover, commands lost */
#define ERROR_TOO_MANY_FLUSH_EVENTS         (0x7D)  /**< @brief flush event queue overflowed; some flush events will not be reported to host properly */
#define ERROR_UNKNOWN_HOST_CHANNEL_ERROR    (0x7E)  /**< @brief there was a HIF buf error, but no overflow or underflow bits were set */

#define ERROR_SAMPLE_RATE                   (0x80)  /**< @brief Invalid sample rate selected */
#define ERROR_LARGE_DECIMATION              (0x81)  /**< @brief The sensor framework was unable to properly decimate to achieve the requested rate. */

#define ERROR_I2C_QUEUE_OVERFLOW            (0x90)  /**< @brief The I2C queue has overflowed. */
#define ERROR_I2C_CALLBACK_ERROR            (0x91)  /**< @brief The I2C transaction completed but the callback was not called as expected */

#define ERROR_TIMER_INSUFFICIENT_RESOURCES  (0xA0)  /**< @brief the Timer library does not have sufficient resources to schedule the timer requested */

#define ERROR_HI_INVALID_GPIO               (0xB0)  /**< @brief An invalid GPIO pin was selected for the host interface. */
#define ERROR_HI_INIT_FAILED                (0xB1)  /**< @brief Error when sending initial meta events */

#define ERROR_CMD_ERROR                     (0xC0)  /**< @brief */
#define ERROR_CMD_TOO_LONG                  (0xC1)  /**< @brief command provided is longer than we support */
#define ERROR_CMD_BUFFER_OVERFLOW           (0xC2)  /**< @brief could not process commands fast enough */

#define ERROR_SYSCALL_INVALID               (0xD0)  /**< @brief An unknown system call was requested. */
#define ERROR_TRAP_INVALID                  (0xD1)  /**< @brief An unknown trap handler was requested */

#define ERROR_FWU_HEADER_CORRUPT            (0xE1)  /**< @brief The firmware header is corrupt. */
#define ERROR_SDI_INVALID_INPUT_STREAM      (0xE2)  /**< @brief An invalid Sensor Data Injection Stream was provided. */
#define ERROR_FWU_UPLOAD_LENGTH_MISMATCH    (0xE3)  /**< @brief The number of bytes uploaded does not match the number of bytes indicated in the firmware headers. */

#define SSBL_RESET_FOR_PRIMARY              (0xE4)  /** < @brief The second-stage bootloader reset the firmware to verify and run primary. */
#define SSBL_PRIMARY_INVALID_COPY_INVALID   (0xE5)  /** < @brief The second-stage bootloader reset to recovery mode due to an error when copying the shadow image to the primary location. */
#define SSBL_PRIMARY_INVALID_SHADOW_INVALID (0xE6)  /** < @brief The second-stage bootloader reset due to verification errors in the the primary and shadow images */
#define SSBL_SHADOW_NEW_COPY_INVALID        (0xE7)  /** < @brief The second-stage bootloader reset to recovery mode due to an error when copying the shadow image to the primary location. */
#define SSBL_SHADOW_INVALID_PRIMARY_INVALID (0xE8)  /** < @brief The second-stage bootloader reset due to verification errors in the the primary and shadow images */
#define SSBL_MAX_FLASH_FW_SIZE_TOO_LARGE    (0xE9)  /** < @brief The specified MAX_FLASH_FW_SIZE does not fit into flash memory. */
#define SSBL_CHAIN_NOT_ALLOWED              (0xEA)  /** < @brief Chained image not allowed for second-stage bootloader. */
#define SSBL_IMAGE_NOT_BUILT_FOR_SSBL       (0xEB)  /** < @brief Image not for use with second-stage bootloader. */
#define ERROR_FWU_IMAGE_BUILT_FOR_SSBL      (0xEC)  /** < @brief Image should only be used with second-stage bootloader. */
#define SSBL_SECTOR_ERROR                   (0xEE)  /** < @brief Second-stage bootloader incorrect sector size. */
#define SSBL_LOGIC_ERROR                    (0xEF)  /** < @brief Second-stage bootloader logic error. */

#define ERROR_ENCRYPTED_IMAGE               (0xF0)  /** < @brief The Encrypted bit was set in the user firmware header, but the kernel does not support encryption. */
#define ERROR_ENCRYPT_NO_SECRET_KEY         (0xF1)  /** < @brief Encryption enabled kernel firmware is being used, but the customer secret key is not set in OTP. */
#define ERROR_ENCRYPT_INVALID_USER_PAYLOAD  (0xF3)  /** < @brief The user payload is encrypted but the size is not a multiple of 16 bytes. */
#define ERROR_ENCRYPT_CRC_MISMATCH          (0xF4)  /** < @brief The CRC-32 of the decrypted user payload does not match the value included at the end of the user payload. */
#define ERROR_PLAINTEXT_IMAGE               (0xF5)  /** < @brief The Encrypted bit was unset in the user firmware header, but the kernel requires encryption. */
#define ERROR_ENCRYPTION_SUPPORT_FLAG       (0xF6)  /** < @brief The EncryptionSupport flag in the kernel payload does not match the expected value. */

#define IRQ_STATE_RUNNING     1     /**< @brief The interrupt routine is running */
#define IRQ_STATE_OFF         0     /**< @brief The interrupt routine has exited */

#define IRQ_STATE_GPIO0       0x10     /**< @brief The GPIO0 interrupt routine is running */
#define IRQ_STATE_GPIO1       0x11     /**< @brief The GPIO1 interrupt routine is running */
#define IRQ_STATE_GPIO2       0x12     /**< @brief The GPIO2 interrupt routine is running */
#define IRQ_STATE_GPIO3       0x13     /**< @brief The GPIO3 interrupt routine is running */
#define IRQ_STATE_GPIO4       0x14     /**< @brief The GPIO4 interrupt routine is running */
#define IRQ_STATE_GPIO5       0x15     /**< @brief The GPIO5 interrupt routine is running */
#define IRQ_STATE_GPIO6       0x16     /**< @brief The GPIO6 interrupt routine is running */

#define IRQ_STATE_I2CMASTER      0x20     /**< @brief The I<sup>2</sup>C master interrupt routine is running */

#define IRQ_STATE_I2CSLAVE_READ  0x30     /**< @brief The I<sup>2</sup>C slave (read) interrupt routine is running */
#define IRQ_STATE_I2CSLAVE_WRITE 0x31     /**< @brief The I<sup>2</sup>C slave (write) interrupt routine is running */

#define IRQ_STATE_TIM2        0x42     /**< @brief The Timer 2 interrupt routine is running */
#define IRQ_STATE_TIM3        0x43     /**< @brief The Timer 3 interrupt routine is running */
#define IRQ_STATE_TIM4        0x44     /**< @brief The Timer 4 interrupt routine is running */

#define IRQ_STATE_SWI0        0x50     /**< @brief The software interrupt 0 routine is running */
#define IRQ_STATE_SWI1        0x51     /**< @brief The software interrupt 1 routine is running */
#define IRQ_STATE_SWI2        0x52     /**< @brief The software interrupt 2 routine is running */
#define IRQ_STATE_SWI3        0x53     /**< @brief The software interrupt 3 routine is running */
#define IRQ_STATE_SWI4        0x54     /**< @brief The software interrupt 4 routine is running */
#define IRQ_STATE_SWI5        0x55     /**< @brief The software interrupt 5 routine is running */





/* Known States */
#define DEBUG_STATE_ENTER     0x00     /**< @brief The current function has been called */
#define DEBUG_STATE_LEAVE     0x0F     /**< @brief The current function is exiting */

/* Modules */
#define DEBUG_STATE_BOOT      0x0
#define DEBUG_STATE_INIT      0x1      /**< @brief Initialization code is running */
#define DEBUG_STATE_INIT_EXT  0x9      /**< @brief Extended states for initialization */

#define DEBUG_STATE_OUTER     0x2      /**< @brief The outerloop code is running */


#define DEBUG_STATE_ALGO         0x3      /**< @brief Algorithm code is running */
#define DEBUG_STATE_ALGO_SENSOR  0x4      /**< @brief Sensor driver code is running */
#define DEBUG_STATE_ALGO_ERROR   0x5      /**< @brief The algorithm error routine is running */
#define DEBUG_STATE_ALGO_KALMAN  0x6      /**< @brief The Kalman filter is running */

#define DEBUG_STATE_HIF          0x7      /**< @brief The host interface */

#define DEBUG_STATE_TINI         0x8      /**< @brief Teardown code is running */

#define DEBUG_STATE_FAIL         0xA      /**< @brief Failed to start properly; halted */

#define DEBUG_STATE_TASK         0xB      /**< @brief Currently executing task */


#include <../7189/includes/debug.h>


#endif /* _INC_DEBUG_H */
