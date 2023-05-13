////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/templates/PhysAccel/PhysAccelSensor.c
///
/// @project    EM7189
///
/// @brief      Template driver for non-composite physical
///             sensor.
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
#include <SensorAPI.h>
#include <Timer.h>
#include <host.h>
#include <bsx_support.h>
#include "PhysAccelSensor.h"

#define DRIVER_REV      (1u)

/*************************************/
/* Global variables                  */
/*************************************/
int32_t phys_accel_sensor_data[3];

/*************************************/
/* Static variables                  */
/*************************************/
static float    phys_accel_sample_rate;
static float    phys_accel_scale_factor;
static uint16_t phys_accel_dynamic_range;
static uint8_t  phys_accel_sample_data[PHYS_ACCEL_SAMPLE_DATA_SIZE];
static SensorPowerMode phys_accel_power_mode;
static sensorSampleDataCallback phys_accel_data_callback;

/*************************************/
/* EM Framework Function Prototypes  */
/*************************************/
static SensorStatus phys_accel_initialize_sensor(PhysicalSensorDescriptor* self);

static SensorPowerMode phys_accel_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor* self);
static SensorPowerMode phys_accel_get_power_mode(PhysicalSensorDescriptor* self);

static float phys_accel_set_sample_rate(float requested_rate, PhysicalSensorDescriptor* self);
static float phys_accel_get_sample_rate(PhysicalSensorDescriptor* self);

static SensorStatus phys_accel_enable_interrupts(PhysicalSensorDescriptor* self);
static SensorStatus phys_accel_disable_interrupts(PhysicalSensorDescriptor* self);

static float phys_accel_get_scale_factor(PhysicalSensorDescriptor* self);
static void  phys_accel_get_sample_data(sensorSampleDataCallback callBack, PhysicalSensorDescriptor* self);

static uint16_t phys_accel_set_dynamic_range(uint16_t desired_range, PhysicalSensorDescriptor* self);
static uint16_t phys_accel_get_dynamic_range(PhysicalSensorDescriptor* self);

/*************************************/
/* Support Function Prototypes       */
/*************************************/
static void phys_accel_reset(PhysicalSensorDescriptor* self);
static SensorPowerMode phys_accel_chip_set_power_mode(Device* device, SensorPowerMode power_mode);
static SensorStatus phys_accel_selftest(Device* device, int16_t offsets[3]);
static void phys_accel_handle_sensor_data(SensorStatus res, void* sensor);

/*************************************/
/* EM Framework Function Definitions */
/*************************************/

//lint -sem( phys_accel_initialize_sensor, thread_protected )
static SensorStatus phys_accel_initialize_sensor(PhysicalSensorDescriptor* self)
{
    uint8_t buffer[1];
    Device* device = &self->device;

    // initialize global variables

    // reset the device
    phys_accel_reset(self);

    // power on the device
    (void)phys_accel_set_power_mode(SensorPowerModeActive, self);

    // read the chip ID
    if (read_data(device, PHYS_ACCEL_CHIP_ID_ADDR, buffer, 1) != SensorOK)
    {
        return SensorErrorNonExistant;
    }

    if (buffer[0] != PHYS_ACCEL_CHIP_ID)
    {
        return SensorErrorUnexpectedDevice;
    }

    // power down the device
    (void)phys_accel_set_power_mode(SensorPowerModePowerDown, self);

    return SensorOK;
}

//lint -sem( phys_accel_set_power_mode, thread_protected )
static SensorPowerMode phys_accel_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor* self)
{
#if 0 // Enable if power mode change may require a rate change
    bool needRateChange = FALSE;
    float sample_rate = 0.0F;
#endif
    Device* device = &self->device;
    int16_t offsets[3] = {0, 0, 0};

    switch (mode)
    {
        case SensorPowerModeSuspend:   // Required
        case SensorPowerModePowerDown: // Required, may duplicate SensorPowerModeSuspend
            (void)phys_accel_chip_set_power_mode(device, SensorPowerModePowerDown);
            break;

        case SensorPowerModeActive:         // Required
        case SensorPowerModeLowPowerActive: // Required, may duplicate SensorPowerModeActive
            (void)phys_accel_chip_set_power_mode(device, SensorPowerModeActive);
            break;

        case SensorPowerModeSelfTest: // Optional, do not support this power mode if not supported by sensor
            {
                SensorStatus result = phys_accel_selftest(device, offsets);

                // Must transition from SensorPowerModeSelfTest to SensorPowerModeSuspend or SensorPowerModePowerDown
                // after self test completes
                (void)self->initialize(self);

                // Must call reportSelfTestResults after transition to SensorPowerModeSuspend or SensorPowerModePowerDown
                reportSelfTestResults(cast_PhysicalToHeader(self), result, offsets[0], offsets[1], offsets[2]);
                return SensorPowerModeSelfTest;
            }

        case SensorPowerModeFOC:             // Optional
        case SensorPowerModeInteruptMotion: // Optional
        case SensorPowerModeOneShot:         // Optional
        default:
            break;
    }
    sensorPowerModeChanged(SensorOK, self);

    // sample rate is affected by power mode
#if 0 // Enable if power mode change may require a rate change
    if(needRateChange)
    {
        (void)phys_accel_set_sample_rate(sample_rate, self);
    }
#endif

    return phys_accel_power_mode;
}

//lint -sem( phys_accel_get_power_mode, thread_protected )
static SensorPowerMode phys_accel_get_power_mode(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return phys_accel_power_mode;
}

//lint -sem( phys_accel_set_sample_rate, thread_protected )
static float phys_accel_set_sample_rate(float requested_rate, PhysicalSensorDescriptor* self)
{
    float newRate = requested_rate;

    // Must place the sensor in the max sample rate supported
    // when a value of 0xFFFF is specified (4.2)
    if ((newRate > self->maxRate) || (newRate == 0xFFFF))
    {
        newRate = self->maxRate;
    }
    // Place the sensor in the min sample rate supported
    // when a value of 1 is specified (4.3)
    if ((newRate < self->minRate) || (newRate == 1))
    {
        newRate = self->minRate;
    }

    // Rates should be a power of two decimation from the max rate (4.11)
    float odr = PHYS_ACCEL_MIN_ODR;

    while (odr < PHYS_ACCEL_MAX_ODR)
    {
        if (odr >= newRate)
        {
            break;
        }
        odr *= 2;
    }
    newRate = odr;

    // Configure physical sensor hardware to desired rate

    // Record current rate
    phys_accel_sample_rate = newRate;

    // Report a rate change (4.9)
    sensorRateChanged(SensorOK, self);

    // Return actual sensor rate (4.4)
    return phys_accel_sample_rate;
}

//lint -sem( phys_accel_get_sample_rate, thread_protected )
static float phys_accel_get_sample_rate(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return phys_accel_sample_rate;
}

static SensorStatus phys_accel_enable_interrupts(PhysicalSensorDescriptor* self)
{
    // Enable interrupts on sensor hardware

    // Set driver flag
    self->int_enabled = TRUE;
    return SensorOK;
}

static SensorStatus phys_accel_disable_interrupts(PhysicalSensorDescriptor* self)
{
    // Disable interrupts on sensor hardware

    // Clear driver flag
    self->int_enabled = TRUE;
    return SensorOK;
}

//lint -sem( phys_accel_get_scale_factor, thread_protected )
static float phys_accel_get_scale_factor(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return phys_accel_scale_factor;
}

//lint -sem( phys_accel_get_sample_data, thread_protected )
static void phys_accel_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor* self)
{
    // cache callback function
    phys_accel_data_callback = callback;

    // Read data from sensor
    read_data_nonblocking(&self->device, PHYS_ACCEL_USER_DATA_ADDR, &phys_accel_sample_data[0],
            PHYS_ACCEL_SAMPLE_DATA_SIZE, phys_accel_handle_sensor_data, self);
}

//lint -sem( phys_accel_set_dynamic_range, thread_protected )
static uint16_t phys_accel_set_dynamic_range(uint16_t desired_range, PhysicalSensorDescriptor* self)
{
    UNUSED(self); // Remove if referenced
    uint16_t check_counter;
    uint8_t index;

    struct ranges
    {
        uint16_t max_range;
        //uint8_t reg_val; // (currently unused)
        float scale_factor;
    };

    static const struct ranges supported_ranges[] =
    {
        { 2,  /* PHYS_ACCEL_RANGE_2G,  */ (0.06104F) },  /* 2G~2G  2^16,  1LSB=0.00006104G  0.06104*/
        { 4,  /* PHYS_ACCEL_RANGE_4G,  */ (0.12207F) },  /*  2 to 4g*/
        { 8,  /* PHYS_ACCEL_RANGE_8G,  */ (0.24414F) },  /*  4 to 8g*/
        { 16, /* PHYS_ACCEL_RANGE_16G, */ (0.48828F) }, /*  8 to 16g*/
    };

    // If the desired range is larger than the max, use the max range
    check_counter = sizeof(supported_ranges) / sizeof(supported_ranges[0]) - 1;
    for (index = 0; index < check_counter; index++)
    {
        if (desired_range <= supported_ranges[index].max_range)
        {
            break;
        }
    }

    // Configure sensor hardware to desired range

    // Record values
    phys_accel_scale_factor = supported_ranges[index].scale_factor;
    phys_accel_dynamic_range = supported_ranges[index].max_range;

    return phys_accel_dynamic_range;
}

//lint -sem( phys_accel_get_dynamic_range, thread_protected )
static uint16_t phys_accel_get_dynamic_range(PhysicalSensorDescriptor* self)
{
    UNUSED(self);
    return phys_accel_dynamic_range;
}

/*************************************/
/*  Support Functions                */
/*************************************/

// Send reset to physical sensor
static void phys_accel_reset(PhysicalSensorDescriptor* self)
{
    UNUSED(self); // Remove if referenced
}

// Configure physical sensor to desired power mode
static SensorPowerMode phys_accel_chip_set_power_mode(Device* device, SensorPowerMode power_mode)
{
    UNUSED(device); // Remove if referenced

    if (power_mode != phys_accel_power_mode)
    {
        switch (power_mode)
        {
            case SensorPowerModeSuspend:
                // power down physical sensor

                break;

            case SensorPowerModeActive:
                // power on physical sensor

                break;

            case SensorPowerModePowerDown:
            case SensorPowerModeSelfTest:
            case SensorPowerModeFOC:
            case SensorPowerModeInteruptMotion:
            case SensorPowerModeOneShot:
            case SensorPowerModeLowPowerActive:
            default:
                return phys_accel_power_mode;
        }

        // Write power configuration to physical sensor

        // Set power mode variable
        phys_accel_power_mode = power_mode;
    }

    return phys_accel_power_mode;
}

// Configure physical sensor to run self test
// If not supported in hardware, do not implement self test mode in driver!
static SensorStatus phys_accel_selftest(Device* device, int16_t offsets[3])
{
    UNUSED(device); // Remove if referenced
    UNUSED(offsets); // Remove if referenced

    // Set up physical sensor to perform self test

    // On failure, return SensorErrorSelfTestFailed
    // On success, return SensorSelfTestPassed
    return SensorSelfTestPassed;
}

//lint -sem( phys_accel_handle_sensor_data, thread_protected )
static void phys_accel_handle_sensor_data(SensorStatus res, void* sensor)
{
    if(res != SensorOK)
    {
        return;
    }

    int32_t x  = ((SInt8) phys_accel_sample_data[1]) * 256;
    x |= phys_accel_sample_data[0];

    int32_t y  = ((SInt8) phys_accel_sample_data[3]) * 256;
    y |= phys_accel_sample_data[2];

    int32_t z  = ((SInt8) phys_accel_sample_data[5]) * 256;
    z |= phys_accel_sample_data[4];

    phys_accel_sensor_data[0] = x;
    phys_accel_sensor_data[1] = y;
    phys_accel_sensor_data[2] = z;

    // Call cached callback
    phys_accel_data_callback(sensor);
}

/*************************************/
/* Physical sensor decriptor         */
/*************************************/
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor phys_accel_phy_descriptor =
{
    .device =
    {
        .options =
        {
            .spi =
            {
                .maxClock = 10000, /* 10 MHz */
                .cpol = 1,
                .cpha = 1,
                .reg_shift = 0,
                .read_pol = 1,
                .read_bit = 7,
            },

            .i2c =
            {
                .hasClockStretching = 0,
                .maxClock = 1000, /* 1 MHz */
            }, //lint !e785
        },

        .interface =
        {
            .options = InterfaceAnySPI, /* Any SPI master*/
        },

        .irqEdge = 1, /* Rising edge*/
    }, //lint !e785

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = 16,
    }, //lint !e785

    .type =
    {
        .value = BSX_INPUT_ID_ACCELERATION,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
    }, //lint !e785

    .initialize = phys_accel_initialize_sensor,

    .set_power_mode = phys_accel_set_power_mode,
    .get_power_mode = phys_accel_get_power_mode,

    .set_sample_rate = phys_accel_set_sample_rate,
    .get_sample_rate = phys_accel_get_sample_rate,

    .enable_interrupts = phys_accel_enable_interrupts,
    .disable_interrupts = phys_accel_disable_interrupts,

    .get_scale_factor = phys_accel_get_scale_factor,
    .get_sample_data = phys_accel_get_sample_data,

    .set_dynamic_range = phys_accel_set_dynamic_range,
    .get_dynamic_range = phys_accel_get_dynamic_range,

    .set_sensor_ctl = NULL,
    .get_sensor_ctl = NULL,

    .sensorData = (void*)phys_accel_sensor_data,

    .maxDynamicRange = PHYS_ACCEL_MAX_DYNAMIC_RANGE,
    .maxRate         = PHYS_ACCEL_MAX_ODR,
    .maxCurrent      = PHYS_ACCEL_MAX_CURRENT,
    .minRate         = PHYS_ACCEL_MIN_ODR,
}; //lint !e785
