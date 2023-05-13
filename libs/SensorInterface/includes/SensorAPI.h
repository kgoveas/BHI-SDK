////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/SensorInterface/includes/SensorAPI.h
///
/// @project    EM7189
///
/// @brief      The following describes the datatypes used throughout the
///             accelerometer, magnetometer, and gyroscope interfaces.
///
/// @classification  Confidential
///
/// @mainpage     Introduction
///
/// @section      Scope Scope of the Document
///
///               The scope of this document is to specify the interfaces
///              available to custom hook and driver developers.
///
/// @section      Documents Related Documents
/// @li           [1] Datasheet<br>
/// @li           [2] I2C-bus specification and user manual, Rev. 4, 13 February 2012.<br>http://www.nxp.com/documents/user_manual/UM10204.pdf
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2012-2018 EM Microelectronic
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

/** @defgroup SENSOR_INT    Sensor Framework Interface */
/** @addtogroup SENSOR_INT
 * @{
 */

#ifndef __SENSOR_API_H_
#define __SENSOR_API_H_

#include <types.h>
#include <hw_versions.h>
#include <debug.h>

//#define MONITOR_TRIGGER_TIMES // enable debug code to help find offending slow sensor driver

/*****************************/
/*       Data Types          */
/*****************************/

struct VirtualSensorDescriptor;
struct PhysicalSensorDescriptor;
struct SensorDescriptorHeader;
typedef struct VirtualSensorDescriptor VirtualSensorDescriptor;
typedef VirtualSensorDescriptor TimerSensorDescriptor;
typedef struct PhysicalSensorDescriptor PhysicalSensorDescriptor;
typedef struct SensorDescriptorHeader SensorDescriptorHeader;

typedef void(*sensorSampleDataCallback)(SensorDescriptorHeader* sensor);

/************************/
/*     Enumerations     */
/************************/
/**
 * @enum SensorPowerMode
 *
 * @brief Possible sensor power modes supported.
 *
 */
typedef enum SensorPowerMode {
    SensorPowerModePowerDown        = 0,        /**< @brief Shut down the sensor. The sensor should not be performing conversions. */
    SensorPowerModeSuspend          = 1,        /**< @brief Enter a low power state, sensor should only detect movement. */

    SensorPowerModeSelfTest         = 16,       /**< @brief Enter a self-test state in the sensor. The sensor should transition to the SensorPowerModePowerDown mode once the self test is complete. Additionally, reportSensorStatus(status, sensor) should be called to report the result. */
    SensorPowerModeFOC              = 17,       /**< @brief Enter a FOC calibration mode in the sensor. The sensor should transition to the SensorPowerModePowerDown mode once the FOC is complete. Additionally, reportSensorStatus(status, sensor) should be called to report the result. */

    SensorPowerModeInteruptMotion   = 64,       /**< @brief Enter a low power state where the sensor interrupts when motion has occurred */
    SensorPowerModeOneShot          = 96,       /**< @brief Begin a single sensor measurement. SensorPowerModeSuspend or lower should be entered automatically after wards. */

    SensorPowerModeLowPowerActive   = 128,      /**< @brief Enter a low power, high noise sensor state. */
    SensorPowerModeActive           = 255,      /**< @brief Enter a high power, low noise sensor state. */
} SensorPowerMode;

/**
 * @enum SensorStatus
 *
 * @brief Sensor status results.
 */
typedef enum SensorStatus{
    /* Valid Responses */
    SensorOK                    = SUCCESS,                  /**< @brief Sensor responded with expected data. */
    SensorInitialized           = INIT_PASS,                /**< @brief Sensor has been initialized. */
    SensorSelfTestPassed        = TEST_PASS,                /**< @brief Sensor self test has passed. */
    SensorFOCPassed             = FOC_PASS,                 /**< @brief Sensor FOC test has passed. */

    /* Error Responses */
    SensorUnknownError          = ERROR_UNKNOWN,            /**< @brief An unknown error has occurred. */
    SensorDataUnavailable       = ERROR_DATA_UNAVAILABLE,   /**< @brief New sensor data was not available, old results returned. */
    SensorDataOverflow          = ERROR_DATA_OVERFLOW,      /**< @brief The sensor result has saturated. */
    SensorErrorNonExistant      = ERROR_NO_DEVICE,          /**< @brief Unable to communicate with sensor, sensor did not ACK. */
    SensorErrorUnexpectedDevice = ERROR_UNEXPECTED_DEVICE,  /**< @brief A different sensor was detected at the address. */

    SensorErrorSlowRate         = ERROR_SLOW_SAMPLE_RATE,   /**< @brief The system has detected that the sensor is not running at the requested rate. */


    SensorErrorSelfTestFailed       = ERROR_SELF_TEST_FAIL,     /**< @brief The sensor self test failed. */
    SensorErrorSelfTestAxisXFailed  = ERROR_SELF_TEST_X_FAIL,   /**< @brief The sensor self test failed due to an issue with axis x. */
    SensorErrorSelfTestAxisYFailed  = ERROR_SELF_TEST_Y_FAIL,   /**< @brief The sensor self test failed due to an issue with axis y. */
    SensorErrorSelfTestAxisZFailed  = ERROR_SELF_TEST_Z_FAIL,   /**< @brief The sensor self test failed due to an issue with axis z. */
    SensorErrorFOCFailed            = ERROR_FOC_FAIL,           /**< @brief The sensor FOC failed. */
    SensorTestUnsupported           = ERROR_TEST_UNSUPPORTED,   /**< @brief Sensor test mode not implemented. */
    SensorBusy                      = ERROR_SENSOR_BUSY,        /**< @brief The sensor is busy and cannot be tested */

    SensorStatusFrameworkReserved = 255,      /**< @brief Reserved for internal use only */
} SensorStatus;

#include <SensorBus.h>

/*****************************/
/*       Macros              */
/*****************************/
// PLACEHOLDER_SENSOR_API
// SENSOR_TEMPERATURE
// SENSORS_MIN_NOISE
// HAS_ACTUAL_RATE
// CONFIG_HAS_FLOAT_MATRIX
// DELAYED_SAMPLE_RATE

// LARGE_DECIMATION
// VIRTUAL_MODE_CHANGE
// VIRTUAL_CURRENT_CONSUMPTION

// NO_DECIMATE_INTEGER


/*****************************/
/*        Sensor Data         */
/*****************************/

/********************************************
 * The Following Sensor ranges have been defined
 * ------------------------------------
 * 0 Host Visible Sensor Type start
 * ...
 * 31 Host Visible Sensor Type end
 * ------------------------------------
 * 32 Customer specific (non-visible) sensor types
 * ...
 * 47 Customer specific (non-visible) sensor types
 * ------------------------------------
 * 48 EM internal sensor types
 * ...
 * 63 EM internal sensor types.
 * ------------------------------------
 ********************************************/

/******* Host Visible Sensor Types **********/
#define SENSOR_TYPE_PAD                                 (0x00)      /**< @brief Signals end of data */
#define SENSOR_TYPE_VISIBLE_BEGIN                       (0x01)      /**< @brief First id of sensor types visible to the host. */

#define SENSOR_TYPE_BSX_RESERVED_START                  (SENSOR_TYPE_VISIBLE_BEGIN) /**< @brief First id of sensor types reserved for BSX use. */

#define SENSOR_TYPE_BSX(__bsx__)                        ((__bsx__) >> 1u)           /**< @brief Translate from BSX Output IDs to Sensor Framework values. */
#define BSX_TYPE(__sensor_type_)                        ((__sensor_type_) << 1u)    /**< @brief Translate from Sensor Framework values to the BSX Output IDs. */

#include "../../BSX/includes/bsx_physical_sensor_identifier.h"
#define NON_BSX_INPUT_ID_BEGIN                          (0x20)      /**< @brief first allowed physical sensor ID beyond those defined by BSX */
#define SENSOR_TYPE_INPUT_STEP_COUNTER                  (0x20)
#define SENSOR_TYPE_INPUT_STEP_DETECTOR                 (0x21)
#define SENSOR_TYPE_INPUT_SIGNIFICANT_MOTION            (0x22)
#define SENSOR_TYPE_INPUT_ANY_MOTION                    (0x23)
#define SENSOR_TYPE_INPUT_EXCAMERA                      (0x24)

#define SENSOR_TYPE_INPUT_GPS                           (0x30)
#define SENSOR_TYPE_INPUT_LIGHT                         (0x31)
#define SENSOR_TYPE_INPUT_PROXIMITY                     (0x32)
#define SENSOR_TYPE_INPUT_EXLED                         (0x33)


#define NON_BSX_INPUT_ID_END                            (0x3F)      /**< @brief last allowed physical sensor ID beyond those defined by BSX */

#include "../../BSX/includes/bsx_virtual_sensor_identifier.h"


#define SENSOR_TYPE_BSX_RESERVED_END                    (0x7F)      /**< @brief Last id of sensor types reserved for BSX use. */

#define SENSOR_TYPE_NON_BSX_VISIBLE_START               (0x80)      /**< @brief for BST-provided virtual sensors like pressure, temperature, gas */
#define SENSOR_TYPE_TEMPERATURE                         (0x80)
#define SENSOR_TYPE_PRESSURE                            (0x81)
#define SENSOR_TYPE_HUMIDITY                            (0x82)
#define SENSOR_TYPE_GAS                                 (0x83)
#define SENSOR_TYPE_WAKE_TEMPERATURE                    (0x84)
#define SENSOR_TYPE_WAKE_PRESSURE                       (0x85)
#define SENSOR_TYPE_WAKE_HUMIDITY                       (0x86)
#define SENSOR_TYPE_WAKE_GAS                            (0x87)
#define SENSOR_TYPE_STEP_COUNTER                        (0x88)
#define SENSOR_TYPE_STEP_DETECTOR                       (0x89)
#define SENSOR_TYPE_SIGNIFICANT_MOTION                  (0x8A)
#define SENSOR_TYPE_WAKE_STEP_COUNTER                   (0x8B)
#define SENSOR_TYPE_WAKE_STEP_DETECTOR                  (0x8C)
#define SENSOR_TYPE_WAKE_SIGNIFICANT_MOTION             (0x8D)
#define SENSOR_TYPE_ANY_MOTION                          (0x8E)
#define SENSOR_TYPE_WAKE_ANY_MOTION                     (0x8F)
#define SENSOR_TYPE_VIRT_EXCAMERA                       (0x90)
#define SENSOR_TYPE_GPS                                 (0x91)
#define SENSOR_TYPE_LIGHT                               (0x92)
#define SENSOR_TYPE_PROXIMITY                           (0x93)
#define SENSOR_TYPE_WAKE_LIGHT                          (0x94)
#define SENSOR_TYPE_WAKE_PROXIMITY                      (0x95)
#define SENSOR_TYPE_ALTITUDE                            (0x96)
#define SENSOR_TYPE_EXLED                               (0x97)
#define SENSOR_TYPE_WAKE_EXLED                          (0x98)

#define SENSOR_TYPE_NON_BSX_VISIBLE_END                 (0x9F)      /**< @brief Last id for BST-provided virtual sensors like pressure, temperature, gas */

#define SENSOR_TYPE_CUSTOMER_VISIBLE_START              (0xA0)      /**< @brief for customer-provided virtual sensors */
#define SENSOR_TYPE_CUSTOMER_VISIBLE_END                (0xBF)      /**< @brief Last id for customer-provided virtual sensors */

#define SENSOR_TYPE_VISIBLE_END                         (0xBF)      /**< @brief Last id of sensor types reserved for normal use. */

/* Sensor ID changes just on Android L host side */
#define SENSOR_TYPE_HOST_VISIBLE_END                    (0xBF)      /**< @brief Last id of sensor types reserved for normal use and visible to the host. */

/******* Customer sensor types **********/
#define SENSOR_TYPE_CUSTOMER_BEGIN                      (0xC0)      /**< @brief First id of sensor types reserved for customer use. */
#define SENSOR_TYPE_VIRT_BSX                            (0xC0)      /**< @brief Fusion engine */
#define SENSOR_TYPE_CUSTOMER_END                        (0xCF)      /**< @brief Last id of sensor types reserved for customer use. */

/******* EM Internal sensor types **********/
#define SENSOR_TYPE_EM_BEGIN                            (0xD0)      /**< @brief First id of sensor types reserved for em use only. */
#define SENSOR_TYPE_HANG_DETECTOR                       (0xD0)      /**< @brief Hang detector */
#define SENSOR_TYPE_EM_END                              (0xDF)      /**< @brief Last id of sensor types reserved for em use only. */

#define SENSOR_TYPE_MAX_DESCRIPTORS                     (SENSOR_TYPE_EM_END + 1) /**< @brief The maximum number of sensor descriptors that can be present in the system. */

/* System events */
#define SENSOR_TYPE_UPPER_START                         (0xE0)      /**< @brief marker for start of non-contiguous types */
#define SENSOR_TYPE_HOST_UPPER_START                    (0xE0)      /**< @brief marker for start of non-contiguous types */

#define SENSOR_TYPE_BSXUPDATE_SUB                       (0xF3)      /**< @brief subscription input and physical config output */
#define SENSOR_TYPE_BSXDOSTEP_INPUT                     (0xF4)      /**< @brief input data to BSX4 for one axis */

#define SENSOR_TYPE_WAKEUP_TIMESTAMP_SMALL_DELTA        (0xF5)      /**< @brief 8-bit delta timestamp */
#define SENSOR_TYPE_WAKEUP_TIMESTAMP_LARGE_DELTA        (0xF6)      /**< @brief 16-bit delta timestamp */
#define SENSOR_TYPE_WAKEUP_TIMESTAMP_FULL               (0xF7)      /**< @brief 40-bit full timestamp */
#define SENSOR_TYPE_WAKEUP_META                         (0xF8)      /**< @brief Event ID / Sensor type for meta events stored in the wakeup FIFO. */

#define SENSOR_TYPE_DEBUG                               (0xFA)      /**< @brief Debug strings or binary data up to 16 bytes of payload; byte 1 bit 7 = ASCII (0), binary (1); bits 0-6 = valid bytes */

#define SENSOR_TYPE_TIMESTAMP_SMALL_DELTA               (0xFB)      /**< @brief 8-bit delta timestamp */
#define SENSOR_TYPE_TIMESTAMP_LARGE_DELTA               (0xFC)      /**< @brief 16-bit delta timestamp */
#define SENSOR_TYPE_TIMESTAMP_FULL                      (0xFD)      /**< @brief 40-bit full timestamp */
#define SENSOR_TYPE_META                                (0xFE)      /**< @brief Event ID / Sensor type for meta events stored in the non-wakeup FIFO (or the only FIFO in the case of KitKat). */
#define SENSOR_TYPE_FILLER                              (0xFF)      /**< @brief special value that should be skipped; parsing on host should continue */

/******* Meta event IDs **********/
#define META_EVENT_NOP                                  (0x00)      /**< @brief The fifo needed to be padded out by one or more bytes. */
#define META_EVENT_FLUSH_COMPLETE                       (0x01)      /**< @brief The FIFO flush has been completed for the specified sensor */
#define META_EVENT_SAMPLE_RATE_CHANGED                  (0x02)      /**< @brief The specified sensor is now running at a new sample rate. */
#define META_EVENT_POWER_MODE_CHANGED                   (0x03)      /**< @brief The specified sensor has changed power states. */
#define META_EVENT_ERROR                                (0x04)      /**< @brief A system error occurred; this meta event will be sent to the debug FIFO */

#define META_EVENT_ALGORITHM_EVENTS                     (0x05)      /**< @brief Contain algorithm specific meta events. */
#define META_EVENT_SENSOR_STATUS                        (0x06)      /**< @brief Change to sensor status (low, mid, high accuracy). */

#define META_EVENT_BSX_DO_STEPS_MAIN                    (0x07)      /**< @brief Return value from main thread as byte 2, num output samples as byte 3 */
#define META_EVENT_BSX_DO_STEPS_CALIB                   (0x08)      /**< @brief Return value from calib thread as byte 2, num output samples as byte 3 */
#define META_EVENT_BSX_GET_OUTPUT_SIGNAL                (0x09)      /**< @brief Return value from get output signal as byte 2, bsx_id input value as byte 3 */

#define META_EVENT_SENSOR_ERROR                         (0x0B)      /**< @brief Error occurred in physical sensor */

#define META_EVENT_FIFO_OVERFLOW                        (0x0C)      /**< @brief The system FIFO has overflowed. Old data is being overwritten with new data. */
#define META_EVENT_DYNAMIC_RANGE_CHANGED                (0x0D)      /**< @brief The specified sensor is now running with a new dynamic range */
#define META_EVENT_FIFO_WATERMARK                       (0x0E)      /**< @brief watermark was reached in the FIFO */

#define META_EVENT_INITIALIZED                          (0x10)      /**< @brief The system has completed it's initialization sequence */
#define META_EVENT_TRANSFER_CAUSE                       (0x11)      /**< @brief Event prepended to FIFO to indicate which sensor caused the start of the transfer; sensor_id = the sensor, extra_info = intstat register */
#define META_EVENT_SENSOR_FRAMEWORK                     (0x12)      /**< @brief used to report events related to the sensor framework */
#define META_EVENT_RESET                                (0x13)      /**< @brief why we were most recently reset */
#define META_EVENT_SPACER                               (0x14)      /**< @brief spacer */

#define SENSOR_FRAMEWORK_DELAYED_TRIGGER                (0x01)      /**< @brief The sensor framework was unable trigger due to a delayed trigger pending. */
#define SENSOR_FRAMEWORK_DROPPED_TRIGGER                (0x02)      /**< @brief The sensor framework delayed the trigger due to a currently running trigger. */
#define SENSOR_FRAMEWORK_DISABLED_HANG_DETECTION        (0x03)      /**< @brief The sensor framework disabled hang detection on the sensor due slow requested rate. */
#define SENSOR_FRAMEWORK_PARENT_DISABLED                (0x04)      /**< @brief The sensor framework detected that an enabled virtual driver did not have the parent enabled. */

#define PHYSICAL_SENSOR_DESCRIPTOR __attribute__ ((section (".phys_sensor_descriptors")))
#define VIRTUAL_SENSOR_DESCRIPTOR  __attribute__ ((section (".virt_sensor_descriptors")))
#define TIMER_SENSOR_DESCRIPTOR    __attribute__ ((section (".timer_sensor_descriptors")))

#define DRIVER_TYPE_VIRTUAL     0x00
#define DRIVER_TYPE_TIMER       0x40
#define DRIVER_TYPE_PHYS        0x80


/*****************************/
/*       Functions           */
/*****************************/


#if defined(MONITOR_TRIGGER_TIMES)
#define MTT_DEPTH 4 // number of previous trigger durations to remember

typedef struct
{
    UInt32 start_time;
    UInt32 stop_time;
    UInt32 duration_history[MTT_DEPTH];
    UInt32 index;       // location in the duration history in which to insert next value
    UInt32 sum;         // running sum used to compute average
    UInt32 average;     // sum / count
    UInt32 count;       // number of triggers so far
    UInt32 minimum;     // shortest trigger so far
    UInt32 maximum;     // longest trigger so far
} MonitorTrigger;

extern void monitorReset(void);         // start or restart taking statistics
extern void monitorLog(void);           // dump statistics for all drivers to FIFO
extern void requestMonitorLog(void);    // call this to request a log at the next main loop call to handleTriggeredSensors()
extern void monitorTriggerStart(MonitorTrigger *mon); // begin measuring run time for get_sample_data() or handle_sensor_data()
extern void monitorTriggerStop(MonitorTrigger *mon); // finish measuring run time and record stats

#endif


#define DRIVER_TYPE_VIRTUAL_FLAG     0x00
#define DRIVER_TYPE_TIMER_FLAG       0x01
#define DRIVER_TYPE_PHYSICAL_FLAG    0x02
typedef struct
{
    UInt8   id;                         /**< @brief The current driver id of the sensor. */
    UInt8   version;                    /**< @brief The current driver revision of the sensor */

    UInt8 resolution;                   /**< @brief The resolution of the sensor input data. */
    UInt8 disabled;                     /**< @brief The sensor has been disabled due to an error. */

    struct {
        UInt8   triggered;            /**< @brief The sensor has been scheduled to run. */
        UInt8   enabled;              /**< @brief The sensor has been enabled. */
        UInt8   i2c_nack;             /**< @brief The sensor has had an i2c communication error. */
        UInt8   devid_error;          /**< @brief The sensor was not detected at the supplied address */
        UInt8   transient_error;      /**< @brief The sensor had a transient error. */
        UInt8   list_ran;             /**< @brief The timer sensor has run since it's been triggered, but it still need to trigger another priority level. */
        UInt8   rate_changed;         /**< @brief The host interface has updated the lastRequestedRate entry. */
        UInt8   range_changed;        /**< @brief The host interface has updated the lastRequestedRange entry. */

        UInt8   pending_trigger;      /**< @brief The sensors trigger has not finished running, but a new trigger has arrived. */

        UInt8   enabled_ack;          /**< @brief The sensor framework has ACKed that the driver is enabled. */
        UInt8   just_turned_on;       /**< @brief Signal from physical sensor to associated virtual sensor; used to trigger an on-change sensor to output its current value */
        UInt8   self_disabled;        /**< @brief Needed to turn off asynchronously, e.g., from a one-shot sensor; alias of rsvd2 in di01 and di02 ROMs */
    } status;

    UInt32 pad0;
} SensorInfo;

typedef struct SensorType
{
    UInt8 value;                      /**< @brief Sensor Type (Sensor ID) */
    UInt8 flags:2;                    /**< @brief Sensor type flags - Virtual, Timer, or Physical */
    UInt8 wakeup_ap:1;                /**< @brief This virtual sensor data is placed in the wakeup fifo and can wakeup the host. */
    UInt8 no_hang:1;                  /**< @brief Disable hang detection code for physical sensors. */
    UInt8 no_decimation:1;            /**< @brief Always use the parent sensor rate, even if a slower rate is requested by the host. */
    UInt8 on_change:1;                /**< @brief This is an on-change sensor, which will require special processing when samples are lost. */
    UInt8 always_on:1;                /**< @brief This is used to ensure that the driver is <b>always</b> enabled, even if no children are enabled. */
    UInt8 hidden:1;                   /**< @brief This sensor should be hidden by the host interface */
    UInt8 decimate_integer:1;         /**< @brief If unset, virtual sensors will decimate by a power of two. If set, any integer value will be used instead. */
    UInt8 on_change_map_bit:5;        /**< @brief Which bit in the sensor_data_block on change map for this sensor */
    UInt8 rsvd_flags2:2;
    UInt8 rsvd1;
} SensorType;


typedef union
{
    PTR(PhysicalSensorDescriptor*) phys;
    PTR(VirtualSensorDescriptor*)  virt;
    PTR(SensorDescriptorHeader*)   header;
} SensorDependency;



typedef union
{
    struct
    {
        SensorType type;
    } sensor;

    /* Note: Valid for virtual sensors only */
    struct {
#define DECIMATE_NEVER        0         /**< @brief When the DECIMATE_NEVER mode is used, the timer sensor always runs at the requested rate. */
#define DECIMATE_ALWAYS       1         /**< @brief When the DECIMATE_ALWAYS mode is used, the timer sensors runs at whatever rate the children sensors request up to the rate specified in the timer driver. */
#define DECIMATE_POWER_TWO    2         /**< @brief When the DECIMATE_POWER_TWO mode is used, the timer sensor always runs at the rate specified in the descriptor, or a power of two devision of it (/1, /2, /4, /8, etc). This should be used in most cases. */

        UInt8   decimate:2;             /**< @brief The decimation mode to use for the timer sensor. Valid values are @ref DECIMATE_NEVER, @ref DECIMATE_ALWAYS, and @ref DECIMATE_POWER_TWO. */
        UInt8   decimate_max:6;         /**< @brief Internal use only. Set to 0 in the descriptor and do not modify. */
        SInt8   index;                  /**< @brief Internal use only. Contains the timer index for use with the timer library. */
        UInt16  rate;                   /**< @brief The rate to run the timer library at. The timer will never be faster than this rate, however it may be lower depending on the decimation mode. */
    } timer;



    /* Note: Used for internal firmware */
    PTR(PhysicalSensorDescriptor*) phys;
    PTR(VirtualSensorDescriptor*)  virt;
    PTR(SensorDescriptorHeader*)  header;
} TriggerSource;


struct SensorDescriptorHeader {
    PTR(SensorDescriptorHeader*)    triggerList;    /**< @brief The next sensor to trigger after this one. */
    SensorInfo                      info;
    SensorType                      type;

#if defined(MONITOR_TRIGGER_TIMES)
    MonitorTrigger                  monitor;
#endif
};

struct PhysicalSensorDescriptor {
    PTR(SensorDescriptorHeader*)    triggerList;    /**< @brief The next sensor to trigger after this one. */
    SensorInfo                      info;
    SensorType                      type;

#if defined(MONITOR_TRIGGER_TIMES)
    MonitorTrigger                  monitor;
#endif

    Device                          device;         /**< @brief The Device descriptor containing information on talking to the physical device. */


    /**
     * @brief The initialize function is used to reset the physical device and driver into a known state. Once the initialize
     *          function returns, the driver should be in the @ref SensorPowerModePowerDown state.
     *
     * @param self      The physical sensor driver to initialize.
     *
     * @returns         The state of the sensor driver after initialization.
     */
    FUNC_PTR(initialize,            SensorStatus (*initialize)(PhysicalSensorDescriptor* self));

    FUNC_PTR(set_power_mode,        SensorPowerMode (*set_power_mode)(SensorPowerMode mode, PhysicalSensorDescriptor* self));
    FUNC_PTR(get_power_mode,        SensorPowerMode (*get_power_mode)(PhysicalSensorDescriptor* self));

    FUNC_PTR(set_sample_rate,       float (*set_sample_rate)(float rate, PhysicalSensorDescriptor* self));
    FUNC_PTR(get_sample_rate,       float (*get_sample_rate)(PhysicalSensorDescriptor* self));

    FUNC_PTR(enable_interrupts,     SensorStatus (*enable_interrupts)(PhysicalSensorDescriptor* self));
    FUNC_PTR(disable_interrupts,    SensorStatus (*disable_interrupts)(PhysicalSensorDescriptor* self));

    FUNC_PTR(get_scale_factor,      float (*get_scale_factor)(PhysicalSensorDescriptor* self));
    FUNC_PTR(get_sample_data,       void (*get_sample_data)(sensorSampleDataCallback callback, PhysicalSensorDescriptor* self));

    FUNC_PTR(set_dynamic_range,     UInt16 (*set_dynamic_range)(UInt16 desired_range, PhysicalSensorDescriptor* self));
    FUNC_PTR(get_dynamic_range,     UInt16 (*get_dynamic_range)(PhysicalSensorDescriptor* self));

    FUNC_PTR(set_sensor_ctl,        SensorStatus (*set_sensor_ctl)(UInt8 code, const UInt8 *data, UInt16 length, PhysicalSensorDescriptor* self));
    FUNC_PTR(get_sensor_ctl,        SensorStatus (*get_sensor_ctl)(UInt8 code, UInt8 *data, UInt16 *length, PhysicalSensorDescriptor* self));

    FUNC_PTR(reserved0,             void (*reserved0)());
    FUNC_PTR(reserved1,             void (*reserved1)());
    FUNC_PTR(reserved2,             void (*reserved2)());
    FUNC_PTR(reserved3,             void (*reserved3)());

#if defined(DELAYED_SAMPLE_RATE)
    UInt16                          delayTime; /**< @brief The amount of time, in ticks, to delay before the first sample */
    /** @cond INTERNAL */
    UInt16                          changedTimestamp;
    /** @endcond */
#endif

    PTR(void*)                      sensorData;
    PTR(void*)                      expansionData;

    SystemTime_t  latestTimestamp;        /**< @brief The latest valid data timestamp for the sensor. */
    SystemTime_t  newTimestamp;           /**< @brief The latest timestamp for the sensor. */

#if defined(SENSOR_TEMPERATURE)
    float   temperature;             /**< @brief the temperature of the sensor driver in deg C. */
#endif

#if defined(SENSORS_MIN_NOISE)
    float   minNoiseFactor;
#endif

    float   maxCurrent;             /**< @brief The maximum current draw of the sensor. units: [mA] */
    float   lastRequestedRate;      /**< @brief The last requested rate for the sensor as determined by the outerloop. */
    float   lastActualRate;         /**< @brief Last actual rate set, this is updated after the sampleRateChanged callback. */

    float   maxRate;                /**< @brief The maximum rate (in Hz) of the sensor. */
    float   minRate;                /**< @brief The minimum rate (in Hz) of the sensor. */

    UInt16  lastRequestedRange;     /**< @brief The last requested range for the sensor. */
    UInt16  maxDynamicRange;        /**< @brief The maximum dynamic range of the sensors in [units] */

    UInt16  defaultDynamicRange;    /**< @brief The default dynamic range to use for this sensor if not request from the host. */
    UInt8   resetDivisorCount;      /**< @brief The number of sample events needed before a sensor is considered alive. */
    UInt8   resetDivisorTimeout;    /**< @brief The number of 30Hz timer intervals to wait before checking for a hung sensor. */

    // Hang detection variables
    UInt8   sampleCount;            /**< @brief The number of sample events since the last hang check. */
    UInt8   resetCount;             /**< @brief The number of recent reset events for the sensor. */
    UInt8   resetCountLimit;
    UInt8   needsReset;             /**< @brief Flag signaling that the sensor should be reset. */

    SInt8   CalMatrix[9];           /**< @brief The 3 axis calibration matrix to apply to 3 axis sensor data. */

    UInt8   numAxis:7;              /**< @brief The number of axis in the sensor. */
    UInt8   int_enabled:1;          /**< @brief The sensor's interrupt or timer is enabled */

    UInt8   sensorControlCode:7;    /**< @brief Sensor control parameter code */
    UInt8   sensorControlDir:1;     /**< @brief Sensor control parameter direction */

#if defined(SENSORS_MIN_NOISE)
#define NOISE_MODE_AVERAGE          (0)
#define NOISE_MODE_MEASURE          (1)
    UInt8   noiseMode:1;            /**< @brief The mode used to specify noise. Valid values: NOISE_MODE_AVERAGE and NOISE_MODE_MEASURE */
    UInt8   rsvd8:7;
#endif

} ALIGNED(8);



struct VirtualSensorDescriptor
{
    PTR(VirtualSensorDescriptor*)   triggerList;    /**< @brief The next sensor to trigger after this one. */
    SensorInfo                      info;
    SensorType                      type;

#if defined(MONITOR_TRIGGER_TIMES)
    MonitorTrigger                  monitor;
#endif

    TriggerSource                   triggerSource;  /**< @brief The trigger source of the driver */

    TriggerSource                   physicalSource; /**< @brief The physical source that primarily affects this sensors data. */
#if defined(SENSOR_TEMPERATURE)
    TriggerSource                   temperatureSource; /**< @brief The sensor driver to use for temperature data. **/
#else
    UInt32 pad64;                   /**< @brief Padding needed to ensure the timestamp is 64bit aligned. Fixes an issue with stuffelf on 64bit hosts. */
#endif

    SystemTime_t  timestamp;        /**< @brief The timestamp of the source causing the driver to be triggered. */

    float   maxRate;                /**< @brief The maximum rate (in Hz) of the sensor. */
    float   minRate;                /**< @brief The minimum rate (in Hz) of the sensor. */

    UInt16  decimationLimit;        /**< @brief Number of decimation counts before sensor can be triggered. */
    UInt16  decimationCount;        /**< @brief Current decimation value of sensor. */
    UInt16  reserved16;
    UInt8   priority;               /**< @brief The priority level to run the virtual sensor calculations at. */
    UInt8   outputPacketSize;       /**< @brief The number of bytes in the sensor output packet, excluding event id. */


    float   lastRequestedRate;      /**< @brief The last requested rate for the sensor as determined by the host interface. */
    float   prevRequestedRate;      /**< @brief The previous requested rate for the sensor. */
    UInt32  lastLatencyMs;          /**< @brief The last requested host interface latency */

    UInt16  lastRequestedRange;     /**< @brief The last requested range for the sensor as determined by the host interface. */
    UInt8   lastStatus;             /**< @brief For some 3 axis sensors, save previous status value to compare against */
    UInt8   pad0;

    FUNC_PTR(initialize,            SensorStatus (*initialize)(VirtualSensorDescriptor* self));
    FUNC_PTR(handle_sensor_data,    SensorStatus (*handle_sensor_data)(VirtualSensorDescriptor* self, void* data));
    FUNC_PTR(get_last_sensor_data,  SensorStatus (*get_last_sensor_data)(VirtualSensorDescriptor *self));

    FUNC_PTR(mode_changed,          void (*mode_changed)(VirtualSensorDescriptor *self, SensorPowerMode mode));

    FUNC_PTR(reserved0,             void (*reserved0)());

    PTR(void*)                      triggeredData; /**< @brief Internal pointer used by the framework to transfer data to drivers */

    union {
        PTR(void*)                      ptr;
        UInt32                          u32;
        SInt32                          s32;
        float                           f32;
    } expansionData;

    PTR(SensorDescriptorHeader*)    nextTriggeredList;
    FUNC_PTR(reserved1,             void (*reserved1)());
    FUNC_PTR(reserved2,             void (*reserved2)());
    FUNC_PTR(reserved3,             void (*reserved3)());

} ALIGNED(8);



/********************************/
/* Typecasting Prototypes       */
/********************************/

/**
 * @fn static PhysicalSensorDescriptor* cast_HeaderToPhysical(SensorDescriptorHeader* header)
 *
 * @brief       Typecasts the SensorDescriptorHeader to a PhysicalSensorDescriptor.
 *
 *  This function does <b>not</b> do any type checking. You should only call this function
 *      if you are certain that the input is actually a physical sensor descriptor.
 *
 * @param       header        The header to typecast.
 * @returns     header, typecasted to a physical sensors.
 */
static ALWAYS_INLINE PhysicalSensorDescriptor* cast_HeaderToPhysical(SensorDescriptorHeader* header)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((PhysicalSensorDescriptor**)(&header));
#pragma pop(BEHAVED)
#else
    return (PhysicalSensorDescriptor*)header;
#endif
}

static ALWAYS_INLINE const PhysicalSensorDescriptor* cast_ConstHeaderToPhysical(const SensorDescriptorHeader* header)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((PhysicalSensorDescriptor**)(&header));
#pragma pop(BEHAVED)
#else
    return (PhysicalSensorDescriptor*)header;
#endif
}

/**
 * @fn static SensorDescriptorHeader* cast_PhysicalToHeader(PhysicalSensorDescriptor* physical)
 *
 * @brief       Typecasts the PhysicalSensorDescriptor to a SensorDescriptorHeader.
 *
 * @param       physical        The physical sensor to typecast.
 * @returns     physical, typecasted to a sensors header.
 */
static ALWAYS_INLINE SensorDescriptorHeader* cast_PhysicalToHeader(PhysicalSensorDescriptor* physical)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((SensorDescriptorHeader**)(&physical));
#pragma pop(BEHAVED)
#else
    return (SensorDescriptorHeader*)physical;
#endif
}

static ALWAYS_INLINE const SensorDescriptorHeader* cast_ConstPhysicalToHeader(const PhysicalSensorDescriptor* physical)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((const SensorDescriptorHeader**)(&physical));
#pragma pop(BEHAVED)
#else
    return (const SensorDescriptorHeader*)physical;
#endif
}

/**
 * @fn static VirtualSensorDescriptor* cast_HeaderToVirtual(SensorDescriptorHeader* header)
 *
 * @brief       Typecasts the SensorDescriptorHeader to a PhysicalSensorDescriptor.
 *
 *  This function does <b>not</b> do any type checking. You should only call this function
 *      if you are certain that the input is actually a virtual sensor descriptor.
 *
 * @param       header        The header to typecast.
 * @returns     header, typecasted to a virtual sensors.
 */
static ALWAYS_INLINE VirtualSensorDescriptor* cast_HeaderToVirtual(SensorDescriptorHeader* header)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((VirtualSensorDescriptor**)(&header));
#pragma pop(BEHAVED)
#else
    return (VirtualSensorDescriptor*)header;
#endif
}

static ALWAYS_INLINE const VirtualSensorDescriptor* cast_ConstHeaderToVirtual(const SensorDescriptorHeader* header)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((const VirtualSensorDescriptor**)(&header));
#pragma pop(BEHAVED)
#else
    return (const VirtualSensorDescriptor*)header;
#endif
}

/**
 * @fn static SensorDescriptorHeader* cast_VirtualToHeader(VirtualSensorDescriptor* virt)
 *
 * @brief       Typecasts the VirtualSensorDescriptor to a SensorDescriptorHeader.
 *
 * @param       virt        The virtual sensor to typecast.
 * @returns     virt, typecasted to a sensors header.
 */
static ALWAYS_INLINE SensorDescriptorHeader* cast_VirtualToHeader(VirtualSensorDescriptor* virt)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((SensorDescriptorHeader**)(&virt));
#pragma pop(BEHAVED)
#else
    return (SensorDescriptorHeader*)virt;
#endif
}

static ALWAYS_INLINE const SensorDescriptorHeader* cast_ConstVirtualToHeader(const VirtualSensorDescriptor* virt)
{
#ifdef _ARCVER
#pragma Off(BEHAVED)
    return *((const SensorDescriptorHeader**)(&virt));
#pragma pop(BEHAVED)
#else
    return (const SensorDescriptorHeader*)virt;
#endif
}


/* Sensor Descriptor Prototypes */
/********************************/

/**
 * @fn static bool reportSensorEvent(const VirtualSensorDescriptor* sensor, const void *data, SystemTime_t timestamp);
 *
 * @brief       Reports the actual rate of the sensor to the host.
 *
 *  This function is called to generate an sensor data event and to report it to
 *      the host interface. In the event that a sensor was enabled by the framework,
 *
 * @param       sensor      The sensor driver to report the event for.
 * @param       data        The event packet to output.
 * @param       timestamp   The timestamp of the event.
 */
//lint -esym(534, reportSensorEvent)
static bool ALWAYS_INLINE reportSensorEvent(const VirtualSensorDescriptor* sensor, const void *data, SystemTime_t timestamp)
{
    extern bool JLI reportEvent(const VirtualSensorDescriptor *p, const void* data);
    return reportEvent(sensor, data);
}

/**
 * @fn void sensorRateChanged(SensorStatus stat, void* sensor);
 *
 * @brief       Reports the actual rate of the sensor to the framework.
 *
 *  This function <b>must</b> be called after a sensor rate has been changed. The
 *      ideal way to do this is to use the sensorRateChanged function as the i2c interrupt
 *      callback with the callback data being the sensor descriptor.
 *
 * @param       stat        The status of the sensor triggering the event.
 * @param       sensor      The sensor driver to report the event for.
 */
void JLI sensorRateChanged(SensorStatus stat, void* sensor);

/**
 * @fn void sensorRangeChanged(SensorStatus stat, void* sensor);
 *
 * @brief       Reports the actual range of the sensor to the framework.
 *
 *  This function <b>must</b> be called after a sensor dynamic range has been changed. The
 *      ideal way to do this is to use the sensorRangeChanged function as the i2c interrupt
 *      callback with the callback data being the sensor descriptor.
 *
 * @param       stat        The status of the sensor triggering the event.
 * @param       sensor      The sensor driver to report the event for.
 */
void JLI sensorRangeChanged(SensorStatus stat, void* sensor);

/**
 * @fn void sensorPowerModeChanged(SensorStatus stat, void* sensor);
 *
 * @brief       Reports the actual rate of the sensor to the host.
 *
 *  This function <b>must</b> be called after a sensor power mode has been changed. The
 *      ideal way to do this is to use the sensorRateChanged function as the i2c interrupt
 *      callback with the callback data being the sensor descriptor.
 *
 * @param       stat        The status of the sensor triggering the event.
 * @param       sensor      The sensor driver to report the event for.
 */
void JLI sensorPowerModeChanged(SensorStatus stat, void* sensor);

/**
 * @fn void forceSensorCheck(void);
 *
 * @brief       Forces the sensor framework to perform a sensor state check and
 *                  perform any rate and power mode changes required.
 *                  Note: this check will not happen immediately.
 */
void JLI forceSensorCheck(void);


/**
 * @fn PhysicalSensorDescriptor* getPhysicalSensorDescriptorByType(UInt32 type, PhysicalSensorDescriptor* phys, UInt32 numPhys);
 *
 * @brief       Locates the physical sensor descriptor corresponding to the requested type.
 *
 * @param       type        The sensor type to locate.
 * @param       phys        The list of physical sensor drivers enabled on the system.
 * @param       numPhys     The number of physical sensor drivers enabled on the system.
 */
PhysicalSensorDescriptor* JLI getPhysicalSensorDescriptorByType(UInt32 type, PhysicalSensorDescriptor* phys, UInt32 numPhys);

/**
 * @fn VirtualSensorDescriptor* getVirtOrTimerSensorDescriptorByType(UInt32 type);
 *
 * @brief       Locates the virtual sensor descriptor corresponding to the requested type.
 *
 *  Internally, this function uses the list of sensor drivers returned using
 *  the getTimerSensorDescriptors and getVirtualSensorDescriptors API functions.
 *
 * @param       type        The sensor type to locate.
 */
VirtualSensorDescriptor* JLI getVirtOrTimerSensorDescriptorByType(UInt32 type);

/**
 * @fn static ALWAYS_INLINE VirtualSensorDescriptor* getVirtualSensorDescriptorByType(UInt32 type, VirtualSensorDescriptor* virt, UInt32 numVirt);
 *
 * @brief       Locates the virtual sensor descriptor corresponding to the requested type.
 *
 * @param       type        The sensor type to locate.
 * @param       virt        The list of virtual sensor drivers enabled on the system.
 * @param       numVirt     The number of virtual sensor drivers enabled on the system.
 */
static ALWAYS_INLINE VirtualSensorDescriptor* DEPRECATED("Please use the getVirtOrTimerSensorDescriptorByType function instead.") getVirtualSensorDescriptorByType(UInt32 type, VirtualSensorDescriptor* virt, UInt32 numVirt)
{
    UNUSED(virt);
    UNUSED(numVirt);
    return getVirtOrTimerSensorDescriptorByType(type);
}

/**
 * @fn SensorDescriptorHeader* getSensorParent(const SensorDescriptorHeader* sensor);
 *
 * @brief       Locates the parent sensor descriptor that causes the input sensor to be triggered.
 *
 * @param       sensor      The sensor descriptor to find the parent of.
 * @retval      NULL        The sensor descriptor has no parent - it is either a physical or timer sensors.
 * @retval     Non-NULL    The immediate parent of the input sensor descriptor.
 */
SensorDescriptorHeader* JLI getSensorParent(const SensorDescriptorHeader* sensor);

/**
 * @fn SensorDescriptorHeader* getSensorSource(SensorDescriptorHeader* sensor);
 *
 * @brief       Locates the primary sensor descriptor that causes the input sensor to be triggered.
 *
 * @param       sensor      The sensor descriptor to find the source of.
 * @retval      NULL        The sensor descriptor has no source - it is either a physical or timer sensors.
 * @retval     Non-NULL    The primary source of the input sensor descriptor.
 */
SensorDescriptorHeader* getSensorSource(SensorDescriptorHeader* sensor);

/**
 * @fn SensorDescriptorHeader* getSensorPhysicalSource(SensorDescriptorHeader* sensor)
 *
 * @brief       Locates the sensor descriptors physical source.
 *
 * @param       sensor      The sensor descriptor to find the source of.
 * @retval      NULL        The sensor descriptor has no physical source.
 * @retval     Non-NULL    The physical source of the input sensor descriptor.
 */
SensorDescriptorHeader* getSensorPhysicalSource(SensorDescriptorHeader* sensor);

#if defined(SENSOR_TEMPERATURE)
/**
 * @fn float getSensorTemperature(SensorDescriptorHeader* sensor);
 *
 * @brief       Returns the temperature of the specified sensor.
 *
 * @param       sensor      The sensor descriptor to find the temperature of.
 * @returns     The sensors temperature in deg C.
 */
float JLI getSensorTemperature(SensorDescriptorHeader* sensor);
#endif

/**
 * @fn UInt8 getPriorityLevel(SensorDescriptorHeader* sensor);
 *
 * @brief       Returns the priority level that the sensor will run at.
 *
 * @param       sensor      The sensor to find the priority of.
 * @retval      PRIORITY_2  The sensor will run at interrupt priority level 2 when handling sensor data.
 * @retval      PRIORITY_M  The sensor will run in the main execution routine..
 */
UInt8 JLI getPriorityLevel(SensorDescriptorHeader* sensor);

#if 0
/**
 * @fn UInt32 getSensorDepth(SensorDescriptorHeader* sensor);
 *
 * @brief       Determines the number of sensors that must be triggers before this sensor is executed.
 *
 * @param       sensor      The sensor to find the depth of.
 * @returns    The number of sensors that must be triggered before this sensor is triggered.
 */
UInt32 getSensorDepth(SensorDescriptorHeader* sensor);
#endif

/**
 * @fn UInt32 getSensorListLength(SensorDescriptorHeader const* sensor);
 *
 * @brief       Determines the number of sensors on the trigger list.
 *
 * @param       sensor      The sensor to find the trigger list length of.
 * @returns    The number of sensors on the trigger list.
 */
UInt32 getSensorListLength(SensorDescriptorHeader const* sensor);

/**
 * @fn UInt32 getSampleRate(SensorDescriptorHeader* sensor);
 *
 * @brief       Determines the sample rate of the sensor - decimated from it's primary source.
 *
 * @param       sensor      The sensor to find the depth of.
 * @returns     The sample rate of the sensor, in Hz.
 */
float JLI getSampleRate(SensorDescriptorHeader* sensor);

/**
 * @fn UInt32 getLastRate(SensorDescriptorHeader* sensor);
 *
 * @brief       Returns the cached sample rate of the sensor - decimated from it's primary source.
 *
 * @param       sensor      The sensor to find the depth of.
 * @returns     The sample rate of the sensor, in Hz.
 */
float JLI getLastRate(SensorDescriptorHeader* sensor);


/**
 * @fn UInt32 getDynamicRange(SensorDescriptorHeader* sensor);
 *
 * @brief Retrieves the current dynamic range requested of the specified sensor driver.
 *
 * @param sensor        The sensor driver.
 *
 * @returns             The dynamic range requested of the sensor.
 */
UInt32 JLI getDynamicRange(SensorDescriptorHeader* sensor);

/**
 * @fn void getSensorInfo(SensorDescriptorHeader *sensor, UInt8 *pCurrent,
 *                          UInt16 *pRange, UInt16 *pResolution, float *pRate,
 *                          float *pMinRate);
 * @brief Retrieves all info about a sensor's physical source.
 * @param sensor        the sensor driver
 * @param pCurrent      current drawn when on
 * @param pRange        dynamic range
 * @param pResolution   resolution in bits
 * @param pRate         maximum rate in Hz. A timer sensor will always report 0 Hz.
 * @param pMinRate      minimum rate in Hz. If a physical source is present or the trigger source is a physical, the reported value will never be larger than the sources reported max rate.
 */
void JLI getSensorInfo(SensorDescriptorHeader *sensor, UInt8 *pCurrent, UInt16 *pRange, UInt16 *pResolution, float *pRate, float *pMinRate);

/**
 * @fn UInt8  getNumBytesPerEvent(SensorDescriptorHeader* sensor);
 *
 * @brief Retrieves the number of bytes the sensor driver generates when outputting an event.
 *
 * @param sensor        The sensor driver.
 *
 * @returns             The number of bytes in a sensor event..
 */
UInt8  getNumBytesPerEvent(SensorDescriptorHeader* sensor);

/**
 * @fn bool isSensorVisible(SensorDescriptorHeader const* sensor);
 *
 * @brief Determines if the sensor driver is visible to the host.
 *
 * @param sensor        The sensor driver.
 *
 * @retval      TRUE    The sensor driver is visible.
 * @retval      FALSE   The sensor driver is invisible.
 */
bool JLI isSensorVisible(SensorDescriptorHeader const* sensor);

/**
 * @fn bool isSensorEnabled(SensorDescriptorHeader const* sensor);
 *
 * @brief Determines if the sensor driver has been enabled for execution.
 *
 * @param sensor        The sensor driver.
 *
 * @retval      TRUE    The sensor driver is currently enabled.
 * @retval      FALSE   The sensor driver is currently disabled.
 */
//lint -function( fopen(1), isSensorEnabled(1) ) // arg cannot be null
bool isSensorEnabled(SensorDescriptorHeader const* sensor);

/**
 * @brief Determines is the sensor is an on-change sensor
 * @param sensor        The sensor driver.
 *
 * @retval      TRUE    The sensor driver is on-change.
 * @retval      FALSE   The sensor driver is not on-change.
 */
//lint -function( fopen(1), isOnChange(1) ) // arg cannot be null
bool isOnChange(const VirtualSensorDescriptor *sensor);

/**
 * @fn void enableSensor(SensorDescriptorHeader* sensor);
 *
 * @brief Enables the sensor driver. This should <b>only</b> be called from the VirtualSensorsDetermined hook.
 *
 * @param sensor        The sensor driver.
 */
//lint -function( fopen(1), enableSensor(1) ) // arg cannot be null
void enableSensor(SensorDescriptorHeader* sensor);

/**
 * @fn void disableSensor(SensorDescriptorHeader* sensor);
 *
 * @brief Disables the sensor driver. This should <b>only</b> be called from the VirtualSensorsDetermined hook.
 * Please note that you should <b>not</b> explicitly disable sensors, as this will override host requests.
 * Instead, you should simply not touch the sensor driver. This function should not be used unless if you
 * know what you are doing.
 *
 * @param sensor        The sensor driver.
 */
//lint -function( fopen(1), disableSensor(1) ) // arg cannot be null
void disableSensor(SensorDescriptorHeader* sensor);

/**
 * @fn bool isSensorEnabled(SensorDescriptorHeader const* sensor);
 *
 * @brief Determines if the sensor driver has been triggered.
 *
 * @param sensor        The sensor driver.
 *
 * @retval      TRUE    The sensor driver has been triggered and is pending execution.
 * @retval      FALSE   The sensor driver does not need to be executed.
 */
//lint -function( fopen(1), isSensorTriggered(1) ) // arg cannot be null
bool isSensorTriggered(SensorDescriptorHeader const* sensor);

/**
 * @fn void triggerSensors(SensorDescriptorHeader* source);
 *
 * @brief The specified sensor's primary interrupt source has triggered. Schedule all child sensors for execution.
 *
 * @param source        The primary sensor driver.
 */
//lint -function( fopen(1), triggerSensors(1) ) // arg cannot be null
void JLI triggerSensors(SensorDescriptorHeader* source);

/**
 * @fn void triggerSensorsWithData(VirtualSensorDescriptor* source, void* data);
 *
 * @brief The specified sensor's primary interrupt source has triggered. Schedule all child sensors for execution.
 *
 * @param source        The primary sensor driver.
 * @param data          The data to pass to the triggered sensors.
 */
//lint -function( fopen(1), triggerSensors(1) ) // arg cannot be null
void JLI triggerSensorsWithData(VirtualSensorDescriptor* source, void* data);

/**
 * @fn void untriggerSensors(SensorDescriptorHeader* source);
 *
 * @brief The specified sensor's primary interrupt source to unscheduled. Cancel all child sensors for execution.
 *
 * @param source        The primary sensor driver.
 */
//lint -function( fopen(1), triggerSensors(1) ) // arg cannot be null
void JLI untriggerSensors(SensorDescriptorHeader* source);


/**
 * @brief swap two 32 bit values atomically
 * @param ptr       the location in memory to swap with the value
 * @param with      the value to swap
 * @return UInt32   the value that was stored at the pointer
 */
#ifdef _ARCVER
static ALWAYS_INLINE UInt32 xchg32(volatile UInt32 *ptr, UInt32 with)
{
   __asm__ volatile ("   ex %0, [%1]\n\t"
    : "=r" (with),"=r"(ptr)
    : "0" (with), "1" (ptr)
    : "memory" );
   return (with);
}

static ALWAYS_INLINE void* xchgptr(void* *ptr, void* with)
{
   __asm__ volatile ("   ex %0, [%1]\n\t"
    : "=r" (with),"=r"(ptr)
    : "0" (with), "1" (ptr)
    : "memory" );
   return (with);
}
#else
static ALWAYS_INLINE UInt32 xchg32(volatile UInt32 *ptr, UInt32 with)
{
   // NOTE: THIS IS NOT ATOMIC. For PC compilation, this should be fine, however
   // If porting to other hardware, this must be made atomic.
   UInt32 tmp = *ptr;
   *ptr = with;
   with = tmp;
   return (with);
}

static ALWAYS_INLINE void* xchgptr(void* *ptr, void* with)
{
   // NOTE: THIS IS NOT ATOMIC. For PC compilation, this should be fine, however
   // If porting to other hardware, this must be made atomic.
   void* tmp = *ptr;
   *ptr = with;
   with = tmp;
   return (with);
}
#endif

extern UInt32 gSensorsTriggered; // atomically set on a trigger; tested and cleared with the following function

/**
 * @fn static UInt32 getAndClearSensorsTriggered(void);
 *
 * @brief atomically check and clear the boolean sensor trigger
 *        flag.
 *
 * @return UInt32   non-zero if any sensors were triggered
 */
static ALWAYS_INLINE UInt32 getAndClearSensorsTriggered(void)
{
   return xchg32(&gSensorsTriggered, 0);
}


/**
 * @fn PhysicalSensorDescriptor* getPhysicalSensorDescriptors(void);
 *
 * @brief Returns the location of the first physical sensor descriptor in memory.
 *
 * @returns      The address of the first physical sensor descriptor.
 */
PhysicalSensorDescriptor* JLI getPhysicalSensorDescriptors(void);

/**
 * @fn VirtualSensorDescriptor*  getVirtualSensorDescriptors(void);
 *
 * @brief Returns the location of the first virtual sensor descriptor in memory.
 *
 * @returns      The address of the first virtual sensor descriptor.
 */
VirtualSensorDescriptor* JLI getVirtualSensorDescriptors(void);

/**
 * @fn VirtualSensorDescriptor*  getTimerSensorDescriptors(void);
 *
 * @brief Returns the location of the first timer sensor descriptor in memory.
 *
 * @returns      The address of the first timer sensor descriptor.
 */
VirtualSensorDescriptor* JLI getTimerSensorDescriptors(void);

/**
 * @fn UInt8 getNumPhysicalSensorDescriptors(void);
 *
 * @brief Returns the number of physical sensor descriptors in memory.
 *
 * @returns      The number of physical sensor descriptors.
 */
UInt8 JLI getNumPhysicalSensorDescriptors(void);

/**
 * @fn UInt8 getNumVirtualSensorDescriptors(void);
 *
 * @brief Returns the number of virtual sensor descriptors in memory.
 *
 * @returns      The number of virtual sensor descriptors.
 */
UInt8 JLI getNumVirtualSensorDescriptors(void);

/**
 * @fn UInt8 getNumTimerSensorDescriptors(void);
 *
 * @brief Returns the number of timer sensor descriptors in memory.
 *
 * @returns      The number of timer sensor descriptors.
 */
UInt8 JLI getNumTimerSensorDescriptors(void);

/**
 * @fn UInt8 getNumTimerSensorDescriptors(void);
 *
 * @brief Returns the number of timer and virtual sensor descriptors in memory.
 *
 * @returns      The number of sensor descriptors.
 */
UInt8 JLI getNumSensorDescriptors(void);

/**
 * @fn bool registerPhysicalSensorDescriptors(PhysicalSensorDescriptor* physicals, UInt8 numPhys);
 *
 * @brief Notify the SensorInterface of the physical sensor drivers available.
 *
 * @retval      TRUE    The SensorInterface will now use the new values.
 * @retval      FALSE   The SensorInterface will continue to use the old values.
 */
//lint -esym(534,registerPhysicalSensorDescriptors)
bool registerPhysicalSensorDescriptors(PhysicalSensorDescriptor* physicals, UInt8 numPhys);

/**
 * @fn bool registerVirtualSensorDescriptors(VirtualSensorDescriptor* virtuals, UInt8 numVirt, UInt8 numTimer);
 *
 * @brief Notify the SensorInterface of the virtual sensor drivers available.
 *
 * @retval      TRUE    The SensorInterface will now use the new values.
 * @retval      FALSE   The SensorInterface will continue to use the old values.
 */
//lint -esym(534,registerVirtualSensorDescriptors)
bool registerVirtualSensorDescriptors(VirtualSensorDescriptor* virtuals, UInt8 numVirt, UInt8 numTimer);

void JLI reportError(UInt8 error);

void JLI handleTriggeredSensors(UInt8 priority, VirtualSensorDescriptor *virtPriFirst, const VirtualSensorDescriptor *virtPriLast);

float JLI determineDesiredRate(SensorDescriptorHeader* driver);

UInt16 JLI determineDesiredRange(const PhysicalSensorDescriptor* phys);


/**
 * @fn bool void sensorInterruptHandler(SystemTime_t timestamp, PhysicalSensorDescriptor* sensor);
 *
 * @brief Notify the sensor framework that the sensor driver has been triggered.
 *          This function is automatically called for GPIO based physical drivers,
 *          However, when a timer based physical driver or a composite driver is triggered,
 *          the interrupt handle must be called programatically by the developer for any dependent drivers.
 *
 * @param   timestamp   The current timestamp. @ref getSystemTime() should be used here if no other time source is available.
 * @param   sensor      The physical sensor driver to trigger.
 */
void JLI sensorInterruptHandler(SystemTime_t timestamp, PhysicalSensorDescriptor* sensor);



#define CAL_TYPE    SInt8
#define DATA_TYPE   SInt32

void JLI applyCalMatrix(const CAL_TYPE* matrix, const DATA_TYPE* source, const CAL_TYPE* offset, DATA_TYPE* dest);


/**
 * @fn bool sensorDataValid(const PhysicalSensorDescriptor* sensor);
 *
 * @brief Determines if the physical sensor descriptor has provided at least one
 *          sample of data since it was last enabled.
 *
 * @param   sensor      The physical sensor driver to check.
 *
 * @retval      TRUE    The sensor has provided at least one sample.
 * @retval      FALSE   The sensor is invalid, disabled, or has not received any sample yet.
 */
bool sensorDataValid(const PhysicalSensorDescriptor* sensor);

#endif /* __SENSOR_API_H_ */

/** @} */
