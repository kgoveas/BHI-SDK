////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/AccelInject/AccelInject.c
///
/// @project    EM7189
///
/// @brief      Sensor driver for the data injection accel.
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
#include <SensorAPI.h>
#include <Timer.h>
#include <command_processor.h>
#include <SensorDataInjection.h>
#include <host.h>
#include "AccelInject.h"

#define DRIVER_REV      (1u)

#define SDI_DEBUG 0

/* define the same default range with other physical accel */
#define INJECT_ACCEL_DEFAULT_RANGE_G       UINT16_C(8)

/*****************************/
/*        Global Variables   */
/*****************************/
accel_t gAccelInjectInfo;

/*****************************/
/*        Static variables   */
/*****************************/
static sdi_entry_t accel_sdi_entry;

/*****************************/
/*        Functions          */
/*****************************/
/**
 * @fn SensorStatus accel_initialize_sensor(PhysicalSensorDescriptor* self);
 *
 * @brief Initializes the accel sensor into a known state.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @retval SensorOK The sensor has been initialized.
 */
static SensorStatus accel_initialize_sensor(PhysicalSensorDescriptor* self)
{
    SensorStatus status;
    accel_sdi_entry.sensor = self;
    accel_sdi_entry.dataBuffer = (UInt8*)gAccelInjectInfo.accelSample;
    accel_sdi_entry.packetSize = sizeof(gAccelInjectInfo.accelSample);

    status = SDI_initializeSensor(&accel_sdi_entry);

    gAccelInjectInfo.accelDynamicRange = 0;
    gAccelInjectInfo.accelScaleFactor = 0.0;

    return status;
}

/**
 * @fn SensorPowerMode accel_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor* self);
 *
 * @brief If possible, sets the sensor to the requested power mode.
 *
 * @param mode  The requested sensor mode.
 * @param self  Pointer to the physical sensor descriptor.
 * @returns     The actual state the sensor was set to.
 */
static SensorPowerMode accel_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor* self)
{
    switch (mode)
    {
        case SensorPowerModeSelfTest:
        case SensorPowerModeFOC:
        case SensorPowerModeInteruptMotion:
        case SensorPowerModeOneShot:
        case SensorPowerModeLowPowerActive:
        default:
            return gAccelInjectInfo.accelPowerMode;

        case SensorPowerModePowerDown:
        case SensorPowerModeSuspend:
            gAccelInjectInfo.accelPowerMode = SensorPowerModeSuspend;
            break;

        case SensorPowerModeActive:
            gAccelInjectInfo.accelPowerMode = SensorPowerModeActive;
            break;
    }

    sensorPowerModeChanged(SensorOK, self);
    return gAccelInjectInfo.accelPowerMode;
}
/**
 * @fn SensorPowerMode accel_get_power_mode(PhysicalSensorDescriptor* self);
 *
 * @brief Used to retrieve the current power mode of the sensor.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @returns The current power mode of the sensor.
 */
static SensorPowerMode accel_get_power_mode(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return gAccelInjectInfo.accelPowerMode;
}
/**
 * @fn float accel_set_sample_rate(float sample_rate, PhysicalSensorDescriptor* self);
 *
 * @brief Set the sensor measurement rate.
 *
 * @param sample_rate The requested sample rate of the sensor in Hz.
 * @param self Pointer to the physical sensor descriptor.
 * @returns The actual sample rate of the sensor.
 */
static float accel_set_sample_rate(float sample_rate, PhysicalSensorDescriptor* self)
{
    float odr = self->minRate;
    while (odr < self->maxRate)
    {
        if (odr >= sample_rate)
        {
            break;
        }
        odr *= 2;
    }
    if (odr != gAccelInjectInfo.accelSampleRate) //lint !e777
    {
        // update timer interval
        gAccelInjectInfo.accelSampleRate = odr;
        sensorRateChanged(SensorOK, self);
    }
    return gAccelInjectInfo.accelSampleRate;
}

/**
 * @fn float accel_get_sample_rate(PhysicalSensorDescriptor* self);
 *
 * @brief Retrieves the sample rate of the sensor.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @returns The current sample rate of the sensor.
 */
static float accel_get_sample_rate(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return gAccelInjectInfo.accelSampleRate;
}

/**
 * @fn SensorStatus accel_enable_interrupts(PhysicalSensorDescriptor* self);
 *
 * @brief Enables the interrupt generation from the sensor.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @retval SensorOK Interrupts have been enabled for the sensor.
 */
static SensorStatus accel_enable_interrupts(PhysicalSensorDescriptor* self)
{
    self->int_enabled = TRUE;
    return SensorOK;
}

/**
 * @fn SensorStatus accel_disable_interrupts(PhysicalSensorDescriptor* self);
 *
 * @brief Disables the interrupt generation from the sensor.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @retval SensorOK Interrupts have been disabled for the sensor.
 */
static SensorStatus accel_disable_interrupts(PhysicalSensorDescriptor* self)
{
    self->int_enabled = FALSE;
    return SensorOK;
}

/**
 * @fn float accel_get_scale_factor(PhysicalSensorDescriptor* self);
 *
 * @brief Used to convert raw sensor data to calibrated values.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @returns Multiplicative scale factor for conversion of sensor result to common units
 */
static float accel_get_scale_factor(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return gAccelInjectInfo.accelScaleFactor;
}

#if SDI_DEBUG // Used for debugging only
typedef struct
{
    UInt8 sensor;
    UInt8 phys_sensor_id;
    int32_t x;
    int32_t y;
    int32_t z;
    UInt64 timestamp;
    UInt8 sequence;
} LOG_T;
UInt8 accelSamples = 0;
LOG_T idata;

void logIt(UInt8 sensor_type, int32_t x, int32_t y, int32_t z, SystemTime_t ts, UInt8 sequence)
{
    idata.sensor = 0xF4;
    idata.phys_sensor_id = sensor_type;
    idata.x = x;
    idata.y = y;
    idata.z = z;
    idata.timestamp = ts;
    idata.sequence = sequence;
    (void)reportMetaEvent(idata.sensor, &idata.phys_sensor_id, ts, FALSE);
}
#endif

/**
 * @fn void accel_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor* self);
 *
 * @brief Retrieves the current sample data and calls the callback
 *
 * @param callback The function to call after the current sample data has been saved.
 * @param self Pointer to the physical sensor descriptor.
 */
static void accel_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor* self)
{
    int32_t x  = ((SInt8) gAccelInjectInfo.accelSample[1]) * 256;
    x |= gAccelInjectInfo.accelSample[0];

    int32_t y  = ((SInt8) gAccelInjectInfo.accelSample[3]) * 256;
    y |= gAccelInjectInfo.accelSample[2];

    int32_t z  = ((SInt8) gAccelInjectInfo.accelSample[5]) * 256;
    z |= gAccelInjectInfo.accelSample[4];

    gAccelInjectInfo.accelSensorData[0] = x;
    gAccelInjectInfo.accelSensorData[1] = y;
    gAccelInjectInfo.accelSensorData[2] = z;

#if SDI_DEBUG // Debugging only
    accelSamples++;

    logIt(self->info.id,
          gAccelInjectInfo.accelSensorData[0],
          gAccelInjectInfo.accelSensorData[1],
          gAccelInjectInfo.accelSensorData[2],
          (SystemTime_t)self->newTimestamp,
          accelSamples);
#endif

    callback((void*)self);
}

/**
 * @fn UInt16 accel_set_dynamic_range(UInt16 desired_range, PhysicalSensorDescriptor* self);
 *
 * @brief Sets the sensor to the requested dynamic range.
 *
 * @param desired_range  The requested range.
 * @param self           Pointer to the physical sensor descriptor.
 * @returns              The current range of the sensor.
 */
static UInt16 accel_set_dynamic_range(UInt16 desired_range, PhysicalSensorDescriptor* self)
{
    uint32_t i;

    struct ranges
    {
        uint16_t max_range;
        float  scale_factor;
    };

    static const struct ranges supported_ranges[] =
    {
        {2,  (0.06104F)}, /* 2G~2G  2^16,  1LSB=0.00006104G */
        {4,  (0.12207F)}, /* 2 to 4g */
        {8,  (0.24414F)}, /* 4 to 8g */
        {16, (0.48828F)}, /* 8 to 16g */
    };

    for (i = 0; i < sizeof(supported_ranges) / sizeof(supported_ranges[0]) - 1u; i++)
    {
        if (desired_range <= supported_ranges[i].max_range)
        {
            break;
        }
    }

    gAccelInjectInfo.accelScaleFactor =  supported_ranges[i].scale_factor;
    gAccelInjectInfo.accelDynamicRange = supported_ranges[i].max_range;
    sensorRangeChanged(SensorOK, self);
    return gAccelInjectInfo.accelDynamicRange;
}

/**
 * @fn UInt16 accel_get_dynamic_range(PhysicalSensorDescriptor* self);
 *
 * @brief Retrieves the dynamic range of the sensor.
 *
 * @param self Pointer to the physical sensor descriptor.
 * @returns The current dynamic range of the sensor.
 */
UInt16 accel_get_dynamic_range(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return gAccelInjectInfo.accelDynamicRange;
}

PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor accel_inject_descriptor =
{
    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = 16,
    }, //lint !e785

    .type = {
        .value = BSX_INPUT_ID_ACCELERATION,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
        .no_hang = TRUE,
    }, //lint !e785

    .minRate = ACCEL_INJECT_OUTPUT_DATA_RATE_MIN_FREQ,
    .maxRate = ACCEL_INJECT_OUTPUT_DATA_RATE_MAX_FREQ,

    .initialize = accel_initialize_sensor,

    .set_power_mode = accel_set_power_mode,
    .get_power_mode = accel_get_power_mode,

    .set_sample_rate = accel_set_sample_rate,
    .get_sample_rate = accel_get_sample_rate,

    .enable_interrupts = accel_enable_interrupts,
    .disable_interrupts = accel_disable_interrupts,

    .get_scale_factor = accel_get_scale_factor,
    .get_sample_data = accel_get_sample_data,

    .set_dynamic_range = accel_set_dynamic_range,
    .get_dynamic_range = accel_get_dynamic_range,
    .defaultDynamicRange = INJECT_ACCEL_DEFAULT_RANGE_G, /* default range is +/- 8g */

    .sensorData = (void*) gAccelInjectInfo.accelSensorData,
}; //lint !e785
