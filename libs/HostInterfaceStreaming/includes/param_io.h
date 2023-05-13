////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/param_io.h
///
/// @project    EM7189
///
/// @brief
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
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

#ifndef _PARAM_IO_H
#define _PARAM_IO_H

#include <host.h>

#if !defined(PACK)
#define PACK __attribute__((__packed__))
#endif

/*** Parameter Pages ***/
#define PARAM_PAGE_NUM(x)       ((x) & 0x0f)  /**< @brief get page number from page register value */
#define PARAM_PAGE_LEN(x)       (((x) >> 4) & 0x0f) /**< @brief extract just the requested size from page register value */

/*** Parameter Page None ***/

/*** Parameter Page System ***/
#define SYSTEM_PARAM_META_EVENT_CTRL        (1) /**< @brief Meta event control parameter **/
#define SYSTEM_PARAM_WAKEUP_META_EVENT_CTRL (2) /**< @brief Meta event control for the Wakeup FIFO */
#define SYSTEM_PARAM_FIFO_CTRL              (3) /**< @brief FIFO control parameter **/
#define SYSTEM_PARAM_FIRMWARE_ID            (4)
#define SYSTEM_PARAM_TIME                   (5) /**< @brief Host IRQ timestamp, event request timestamp, and current timestamp */
#define SYSTEM_PARAM_FRAMEWORK_STATUS       (6) /**< @brief Sensor framework debug information */

#define SYSTEM_PARAM_STACK_INFO             (16) /**< @brief Debug information related to the stacks */

#define SYSTEM_PARAM_BUFFER_WAKE            (28) /**< @brief get info about the wake FIFO */
#define SYSTEM_PARAM_BUFFER_NONWAKE         (29) /**< @brief get info about the nonwake FIFO */
#define SYSTEM_PARAM_BUFFER_DEBUG           (30) /**< @brief get info about the debug FIFO */
#define SYSTEM_PARAM_VIRT_SENS_PRESENT      (31) /**< @brief physical sensors present (bitmap) */
#define SYSTEM_PARAM_PHYS_SENS_PRESENT      (32) /**< @brief physical sensors present (bitmap) */

#define SYSTEM_PARAM_RAM_BANKS              (121) /**< @brief read-only parameter to determine state of each RAM bank */

#define SYSTEM_PARAM_IRQ_COUNTS             (122) /**< @brief special factory-only debug counters */
#define SYSTEM_PARAM_GPIO_COUNTS            (123) /**< @brief special factory-only GPIO interrupt counters */
#define SYSTEM_PARAM_SWI_COUNTS             (124) /**< @brief special factory-only SWI counters */
#define SYSTEM_PARAM_SWT_INFO               (125) /**< @brief special factory-only SW timer info */

#define SYSTEM_PARAM_PRINTF                 (126) /**< @brief test printf output */

/* Meta Event Control */
static UInt16 ALWAYS_INLINE META_EVENT_SHIFT(UInt8 id) // (((__id__) - 1U) << 1U) meta event ids are 1-based; two bits per event
{
    return (id - 1u) << 1u;
}


//#define META_EVENT_CTRL_MASK(__id__)        (((UInt64)0x03) << (META_EVENT_SHIFT(__id__)))
static UInt64 ALWAYS_INLINE META_EVENT_CTRL_MASK(UInt8 id)
{
    return ((UInt64)(0x03u)) << META_EVENT_SHIFT(id);
}

//#define META_EVENT_ENABLED(__id__)          (((UInt64)0x02) << (META_EVENT_SHIFT(__id__)))
static UInt64 ALWAYS_INLINE META_EVENT_ENABLED(UInt8 id)
{
    return ((UInt64)(0x02u)) << META_EVENT_SHIFT(id);
}

//#define META_EVENT_INT_ENABLED(__id__)      (((UInt64)0x01) << (META_EVENT_SHIFT(__id__)))
static UInt64 ALWAYS_INLINE META_EVENT_INT_ENABLED(UInt8 id)
{
    return ((UInt64)(0x01u)) << META_EVENT_SHIFT(id);
}

/* Fifo Control */
typedef struct PACK ParamFIFOControl
{
    UInt32 WakeupWatermark;
    UInt32 WakeupFIFOSize;
    UInt32 NonWakeupWatermark;
    UInt32 NonWakeupFIFOSize;
    UInt32 StatusFIFOSize;
} ParamFIFOControl;

typedef struct PACK FWHash
{
    UInt8 hash[6];
} FWHash;

typedef struct PACK ParamVersion
{
    UInt16 CustomVersion;
    FWHash EMHash;
    FWHash BSTHash;
    FWHash CustomerHash;
} ParamVersion;

typedef struct PACK ParamTime
{
    UInt32 HostIRQTimestampLSW;
    UInt8 HostIRQTimestampMSB;
    UInt32 CurrentTimestampLSW;
    UInt8 CurrentTimestampMSB;
    UInt32 HostEventIRQTimestampLSW;
    UInt8 HostEventIRQTimestampMSB;
    UInt8 Reserved;            /**< @brief Pad to multiple of 4 */
} ParamTime;

typedef struct PACK DebugFlags
{
    UInt8 WatchDogEn:1;
    UInt8 rsvd:7;
} DebugFlags;

typedef struct PACK ParamDebugConfig
{
    DebugFlags flags;
    UInt8 rsvd0;
    UInt8 rsvd1;
    UInt8 rsvd2;
    UInt8 rsvd3;
    UInt8 rsvd4;
    UInt8 rsvd5;
    UInt8 rsvd6;
    /* 8 bytes total */
} ParamDebugConfig;

typedef struct PACK ParamDebugStatus
{
    DebugFlags flags;
    UInt8 rsvd0;
    UInt8 rsvd1;
    UInt8 rsvd2;
    UInt8 rsvd3;
    UInt8 rsvd4;
    UInt8 rsvd5;
    UInt8 rsvd6;
    UInt8 MaxI2CQueueDepth;
    /* 16 bytes total */
} ParamDebugStatus;

typedef struct PACK VirtPresent
{
    UInt8 map[32]; // allow up to 256, but host can read less
} VirtPresent;

typedef struct PACK ParamRAMBanks
{
    UInt16 iccm_powered_map;    // 0-1 bits 0-8 = ICCM banks; 1 = configured for ICCM and powered on
    UInt16 dccm_powered_map;    // 2-3 bits 0-8 = DCCM banks; 1 = configured for DCCM and powered on
    UInt16 iccm_valid_map;      // 4-5 bits 0-8 = ICCM banks; 1 = configured for ICCM and powered on
    UInt16 dccm_valid_map;      // 6-7 bits 0-8 = DCCM banks; 1 = configured for DCCM and powered on
    UInt8 last_iccm_bank;       // 8
    UInt8 last_dccm_bank;       // 9
    UInt8 num_iccm_banks;       // A
    UInt8 num_dccm_banks;       // B
    UInt8 num_banks_powered;    // C
    UInt8 num_iccm_powered;     // D
    UInt8 num_dccm_powered;     // E
    UInt8 dccm_num;             // F from RegCHPMemconfig
    UInt8 ram_pwr;              //10 from RegCHPPowerconfig
    UInt8 ram_rst_n;            //11 from RegCHPPowerconfig
    UInt8 flash_size;           //12 from RegCHPMemconfig
    UInt8 flash_cfg : 1;        //13 from RegCHPMemconfig
    UInt8 fcc_en : 1;           //   from RegFCCControl
} ParamRAMBanks;

/*** Parameter Page Algorithm ***/

/*** Parameter Page Sensors ***/
#define SENSOR_PARAM(__id__)            (__id__)

typedef struct PACK ParamSensorInformation
{
    UInt8 SensorType;           /**< @brief The sensor type */
    UInt8 DriverID;             /**< @brief Unique driver id used to communicate to the sensor */
    UInt8 DriverVersion;        /**< @brief Current version of the sensor driver */
    UInt8 Current;              /**< @brief Maximum current draw of the sensor [mA]. */
    UInt16 MaxRange;            /**< @brief Maximum range of sensor data in SI units */
    UInt16 Resolution;          /**< @brief Number of bits of resolution of underlying sensor */
    float MaxRate;              /**< @brief Maximum sensor rate supported by the hardware */
    UInt32 FIFOReserved;        /**< @brief FIFO size [samples] reserved for this sensor. */
    UInt32 FIFOMax;             /**< @brief Maximum FIFO size for this sensor [samples]. */
    UInt8 EventBytes;           /**< @brief The number of bytes in an event packet, excluding the event id. */
    float MinRate;              /**< @brief Minimum rate supported by the hardware */
    UInt8  reserved[3];         /**< @brief Pad to multiple of 4 */
} ParamSensorInformation;

typedef struct PACK ParamPhysSensorInformation
{
    UInt8  SensorType;          /**< @brief The sensor type */
    UInt8  DriverID;            /**< @brief Unique driver id used to communicate to the sensor */
    UInt8  DriverVersion;       /**< @brief Current version of the sensor driver */
    UInt8  Current;             /**< @brief Maximum current draw of the sensor [mA]. */
    UInt16 CurrentRange;        /**< @brief Current dynamic range of sensor in SI units */
    UInt8  IRQ:1;               /**< @brief Sensor's DRDY interrupt is enabled */
    UInt8  MBusNum:3;           /**< @brief Master Bus Number (1-3) this is attached to */
    UInt8  MBusIsI2C:1;         /**< @brief This Master Bus is SPI=0, I2C=1 */
    UInt8  PowerMode:3;         /**< @brief Sensor power mode */
    UInt8  SlaveAddr;           /**< @brief 7 bit I2C slave address OR SPI GPIO pin for chip select */
    UInt8  GPIOInt;             /**< @brief GPIO pin for DRDY interrupt */
    float  CurrentRate;         /**< @brief Maximum sensor rate supported by the hardware */
    UInt8  NumAxis;             /**< @brief The number of axis reported by the sensor driver */
    UInt8  axisMapping[5];      /**< @brief Axis remapping matrix (integer). 4 bits per axis. */
    UInt8  reserved;            /**< @brief Pad to multiple of 4 */
} ParamPhysSensorInformation;


typedef struct PACK ParamSensorConfig
{
    float  SampleRate;          /**< @brief The requested sample rate of the sensor [Hz] */
    UInt32 MaxReportLatency;    /**< @brief The maximum amount or time to batch samples for [ms] */
    UInt16 ChangeSensitivity;   /**< @brief The minimum amount of change required before an interrupt is generated */
    UInt16 DynamicRange;        /**< @brief The requested dynamic range of the sensor. Multiple of the reported Resolution for the sensor. */
} ParamSensorConfig;

typedef struct PACK ParamPhysSensorConfig
{
    UInt8  axisMapping[5];      /**< @brief Axis remapping matrix (integer). 4 bits per axis. */
    UInt8  reserved[3];         /**< @brief Reserved for future use. */
} ParamPhysSensorConfig;

typedef struct PACK ParamSensorControl
{
    UInt8 Code:7;               /**< @brief Sensor driver-specific code */
    UInt8 Direction:1;          /**< @brief 1 = host wants to do a Get Parameter to read out this code's data */
    UInt8 Data[];               /**< @brief the data being passed to set_sensor_ctl() or returned by get_sensor_ctl() */
} ParamSensorControl;

/*** Functions - local ***/
#endif /* _PARAM_IO_H */
