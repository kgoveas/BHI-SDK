/*!
 * Copyright (C) Robert Bosch. All Rights Reserved. Confidential.
 *
 *
 * <Disclaimer>
 * Common: Bosch Sensortec products are developed for the consumer goods
 * industry. They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the express
 * understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive
 * systems or any system or device that may lead to bodily harm or property
 * damage if the system or device malfunctions. In addition, Bosch Sensortec
 * products are not fit for use in products which interact with motor vehicle
 * systems.  The resale and/or use of products are at the purchaser's own risk
 * and his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any product use not covered by the parameters of the respective valid product
 * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
 * for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec without
 * delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special: This software module (hereinafter called "Software") and any
 * information on application-sheets (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. The Software and Information is subject to the following terms and
 * conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training. Do
 * not use this Software if you do not have the proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use. No license is granted by implication
 * or otherwise under any patent or patent rights of Bosch. Specifications
 * mentioned in the Information are subject to change without notice.
 *
 * @file        AK09915MagSensor.c
 *
 * @date        12/20/2018
 *
 * @brief       Mag driver of AK09915
 *
 */

/*!
 *@addtogroup ak09915_mag_group
 *@brief
 *@{*/

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <Timer.h>
#include <host.h>
#include <types.h>

/*********************************************************************/
/* local header files */
#include "AK09915MagSensor.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV      (1u)

#define  this                       (self)
#define POLLING_RETRY               (200)
#define POLL_TIMER_INDEX_IDLE       (-1)
#ifndef UNUSED
#define UNUSED(__x__)               ((void)(__x__))
#endif /*~ UNUSED */

#define X_AXIS                      (1 << 0)
#define Y_AXIS                      (1 << 1)
#define Z_AXIS                      (1 << 2)

/*********************************************************************/
/* static variables */
static int32_t g_ak09915_sensor_data[3];
static SensorPowerMode g_cached_power_mode;
static int8_t g_poll_timer_index = POLL_TIMER_INDEX_IDLE;
static float g_cached_sample_rate = AK09915_MIN_ODR;
static sensorSampleDataCallback g_cached_callback_func;
static uint8_t g_read_cache[10] = {0};
static uint8_t g_write_cache[2] = {0};

/*********************************************************************/
/* function declarations */
static void ak09915_timer_proc(SensorStatus status, void *param);

/*********************************************************************/
/* function */

/* Allow deprecated API calls from the initialize function. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/*!
 * @brief Driver initialization handler.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Initiate result
 *    SensorOK                                 The sensor has already been initialized
 *    SensorErrorUnexpectedDevice    The sensor did not return expected results.
 *    SensorErrorNonExistant             No device
 */
static SensorStatus ak09915_initialize(PhysicalSensorDescriptor *self)
{
    uint8_t reg_val[2] = {0};
    uint8_t retry = 0;

    /* Force FPGA pin selection to enable AKM. NOP on asic. */
    FPGA.SensorConfig.bits.mag = FPGA_SENSOR_CONFIG_MAG_AKM09915;

    /* Issue soft reset */
    reg_val[0] = AK09915_CNTL3_BIT_SRST;
    if (write_data(&self->device, AK09915_REG_CNTL3, reg_val, 1) != SensorOK)
    {
        return SensorErrorNonExistant;
    }

    retry = POLLING_RETRY;
    while (retry--)
    {
        (void)delayums(1);
        if (read_data(&self->device, AK09915_REG_CNTL3, reg_val, 1) != SensorOK)
        {
            return SensorErrorNonExistant;
        }
        if (!(reg_val[0] & AK09915_CNTL3_BIT_SRST))
        {
            break;
        }
    }

    if (retry == 0)
    {
        return SensorErrorUnexpectedDevice;
    }

    /* Check who am I */
    if (read_data(&self->device, AK09915_REG_WIA1, reg_val, 2) != SensorOK)
    {
        return SensorErrorNonExistant;
    }

    if ((reg_val[0] != AK09915_COMPANY_ID) || (reg_val[1] != AK09915_DEVICE_ID))
    {
        return SensorErrorUnexpectedDevice;
    }

    /* Set noise suppression level */
    reg_val[0] =  AK09915_CNTL1_NSF_DISABLE;
    if (write_data(&self->device, AK09915_REG_CNTL1, reg_val, 1) != SensorOK)
    {
        return SensorErrorNonExistant;
    }
    /* Schedule a dummy timer */
    if (g_poll_timer_index != POLL_TIMER_INDEX_IDLE)
    {
        timer_unschedule_polling(g_poll_timer_index);
    }
    g_poll_timer_index = timer_schedule_polling(g_cached_sample_rate, ak09915_timer_proc, this);

    return SensorOK;
}

/*!
 * @brief Run self-test.
 *
 * @param[in] device : Current physical sensor
 * @param[out] x       : Offset value of axis X
 * @param[out] y       : Offset value of axis Y
 * @param[out] z       : Offset value of axis Z
 *
 * @return Self-test result.
 */
static SensorStatus ak09915_self_test(const Device *device, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t reg_val[7] = {0};
    uint8_t retry = 200;
    int16_t ts16_data[3] = {0};
    int32_t ts32_data[3] = {0};
    uint8_t tu8_check_result = 0;
    uint8_t test_result = 0;

    /* Set power down mode */
    reg_val[0] = AK09915_CNTL2_POWER_DOWN;
    if (write_data(device, AK09915_REG_CNTL2, reg_val, 1) != SensorOK)
    {
        return SensorErrorSelfTestFailed;
    }
    /* empty read data buffer first, BHYTWO-570 */
    (void)delayums(1);
    if (read_data(device, AK09915_REG_ST1, reg_val, 7) != SensorOK)
    {
        return SensorErrorSelfTestFailed;
    }
    /* Set Self-test mode */
    reg_val[0] = AK09915_CNTL2_POWER_SELFTEST;
    if (write_data(device, AK09915_REG_CNTL2, reg_val, 1) != SensorOK)
    {
        return SensorErrorSelfTestFailed;
    }
    /* Poll DRDY */
    retry = POLLING_RETRY;
    while (retry--)
    {
        (void)delayums(1);
        if (read_data(device, AK09915_REG_ST1, reg_val, 1) != SensorOK)
        {
            return SensorErrorSelfTestFailed;
        }
        if (reg_val[0] & AK09915_ST1_BIT_DRDY)
        {
            break;
        }
    }

    if (retry == 0)
    {
        return SensorErrorSelfTestFailed;
    }
    /* Read data */
    if (read_data(device, AK09915_REG_HXL, reg_val, 6) != SensorOK)
    {
        return SensorErrorSelfTestFailed;
    }

    int16_t mag_x = ((int8_t)reg_val[1]) * 256;
    mag_x |= reg_val[0];
    int16_t mag_y = ((int8_t)reg_val[3]) * 256;
    mag_y |= reg_val[2];
    int16_t mag_z = ((int8_t)reg_val[5]) * 256;
    mag_z |= reg_val[4];

    ts16_data[0] = mag_x;
    ts16_data[1] = mag_y;
    ts16_data[2] = mag_z;

    /* convert from LSB to uT */
    ts32_data[0] = ts16_data[0] * AK09915_MAX_RANGE;
    ts32_data[1] = ts16_data[1] * AK09915_MAX_RANGE;
    ts32_data[2] = ts16_data[2] * AK09915_MAX_RANGE;
    *x = (int16_t)(ts32_data[0] / MAX_SINT16);
    *y = (int16_t)(ts32_data[1] / MAX_SINT16);
    *z = (int16_t)(ts32_data[2] / MAX_SINT16);

    /* Check result */
    if ((ts16_data[0] > AK09915_SELFTEST_MAX_X) || (ts16_data[0] < AK09915_SELFTEST_MIN_X))
    {
        tu8_check_result++;
        test_result |= X_AXIS;
    }
    if ((ts16_data[1] > AK09915_SELFTEST_MAX_Y) || (ts16_data[1] < AK09915_SELFTEST_MIN_Y))
    {
        tu8_check_result++;
        test_result |= Y_AXIS;
    }
    if ((ts16_data[2] > AK09915_SELFTEST_MAX_Z) || (ts16_data[2] < AK09915_SELFTEST_MIN_Z))
    {
        tu8_check_result++;
        test_result |= Z_AXIS;
    }

    if (tu8_check_result >= 2)
    {
        return SensorErrorSelfTestFailed;
    }

    if (test_result & 0x7)
    {
        return ((test_result & X_AXIS) ? SensorErrorSelfTestAxisXFailed : ((test_result & Y_AXIS) ? SensorErrorSelfTestAxisYFailed : SensorErrorSelfTestAxisZFailed));
    }

    return SensorSelfTestPassed;
}

#pragma GCC diagnostic pop

/*!
 * @brief Set power mode handler;
 *    No need to do much for ak09915 since it is in polling mode and most work is done by interrupt operations.
 *
 * @param[in] mode : Power mode to set
 * @param[in] self    : Current physical sensor descriptor
 *
 * @return Power mode in use.
 */
static SensorPowerMode ak09915_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    switch (mode)
    {
        case SensorPowerModeSelfTest:
            reportSelfTestResults(cast_PhysicalToHeader(this), ak09915_self_test(&self->device, &x, &y, &z), x, y, z);
            g_cached_power_mode = SensorPowerModePowerDown;
            sensorPowerModeChanged(SensorOK, this);
            return SensorPowerModeSelfTest;

        case SensorPowerModePowerDown:
        case SensorPowerModeSuspend:
            /* Turn off the timer to save power */
            timer_disable_interrupt(g_poll_timer_index);
            g_cached_power_mode = mode;
            break;

        case SensorPowerModeLowPowerActive:
        case SensorPowerModeActive:
            g_cached_power_mode = SensorPowerModeActive;
            break;

        default:
            return g_cached_power_mode;
    }

    sensorPowerModeChanged(SensorOK, this);

    return g_cached_power_mode;
}

/*!
 * @brief Get power mode
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Power mode in use.
 */
static SensorPowerMode ak09915_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return g_cached_power_mode;
}

/*!
 * @brief Set sample rate handler.
 *
 * @param[in] sample_rate : Desired sample rate
 * @param[in] self              : Current physical sensor descriptor
 *
 * @return Sample rate in use.
 */
static float ak09915_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    /* Should be 200Hz */
    float available_rate = this->maxRate;

    if (sample_rate > this->maxRate)
    {
        sample_rate = this->maxRate;
    }
    if (sample_rate < this->minRate)
    {
        sample_rate = this->minRate;
    }

    /* continuously calculate actual sample rate*/
    while (available_rate / 2 >= sample_rate)
    {
        available_rate /= 2;
    }

    if (g_cached_sample_rate != available_rate)
    {
        if (-1 != timer_update_polling(g_poll_timer_index, available_rate))
        {
            g_cached_sample_rate = available_rate;
        }
    }
    sensorRateChanged(SensorOK, this);

    return g_cached_sample_rate;
}

/*!
 * @brief Get sample rate handler.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Sample rate in use.
 */
static float ak09915_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return g_cached_sample_rate;
}

/*!
 * @brief Timer processing function
 *
 * @param[in] status : Usage unknown
 * @param[in] param : Pointer passed from timer schedule function
 *
 * @return None
 */
static void ak09915_timer_proc(SensorStatus status, void *param)
{
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)param;

    /* dynamic SDR setting */
    g_write_cache[0] = AK09915_CNTL2_POWER_SINGLE;
    if (g_cached_sample_rate < AK09915_ODR_THRESHOLD_FOR_SDR)
    {
        g_write_cache[0] |= AK09915_CNTL2_BIT_SDR;
    }
    write_data_nonblocking(&self->device, AK09915_REG_CNTL2, g_write_cache, 1, NULL, NULL);

    sensorInterruptHandler(getSystemTime(), this);
}

/*!
* @brief Enable interrupt handler; Enable timer in case of polling.
*
* @param[in] self : Current physical sensor descriptor
*
* @return SensorOK on success, other values on failure.
*/
static SensorStatus ak09915_enable_interrupts(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    timer_enable_interrupt(g_poll_timer_index);
    this->int_enabled = TRUE;
    return SensorOK;
}

/*!
 * @brief Disable interrupt handler; Disable timer in case of polling.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return SensorOK on success, other values on failure.
 */
static SensorStatus ak09915_disable_interrupts(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    timer_disable_interrupt(g_poll_timer_index);
    this->int_enabled = FALSE;
    return SensorOK;
}

/*!
 * @brief Get scale factor handler.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Scale factor
 */
static float ak09915_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    /* Fixed at 0.15uT / LSB, scale to 0.1uT */
    return 10.0F * 0.15F;
}

/*!
 * @brief Delayed I2C read result processing
 *
 * @param[in] res        : Execution result
 * @param[out] param : Pointer passed from I2C read function
 *
 * @return None
 */
static void ak09915_delayed_read_proc(SensorStatus res, void *param)
{
    if (res != SensorOK)
    {
        return;
    }
    /* Check data ready */
    if (!(g_read_cache[0] & AK09915_ST1_BIT_DRDY))
    {
        return;
    }
    /* Check DOR bit in single power mode */
    if (g_read_cache[0] & AK09915_ST1_BIT_DOR)
    {
        return; /* data overrun */
    }

    int32_t x = ((int8_t)g_read_cache[2]) * 256;
    x |= g_read_cache[1];
    int32_t y = ((int8_t)g_read_cache[4]) * 256;
    y |= g_read_cache[3];
    int32_t z = ((int8_t)g_read_cache[6]) * 256;
    z |= g_read_cache[5];

    g_ak09915_sensor_data[0] = x;
    g_ak09915_sensor_data[1] = y;
    g_ak09915_sensor_data[2] = z;

    g_cached_callback_func(param);

    /* dynamic SDR setting */
    g_write_cache[0] = AK09915_CNTL2_POWER_SINGLE;
    if (g_cached_sample_rate < AK09915_ODR_THRESHOLD_FOR_SDR)
    {
        g_write_cache[0] |= AK09915_CNTL2_BIT_SDR;
    }
    write_data_nonblocking(&((PhysicalSensorDescriptor *)param)->device, AK09915_REG_CNTL2, g_write_cache, 1, NULL, NULL);
}

/*!
 * @brief Get sample data handler
 *
 * @param[in] callback : Callback function in framework; We need to call it when we got data
 * @param[in] self        : Current physical sensor descriptor
 *
 * @return None
 */
static void ak09915_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor *self)
{
    /* cache callback function */
    g_cached_callback_func = callback;
    /* read out sensor data */
    read_data_nonblocking(&self->device, AK09915_REG_ST1, g_read_cache, 9, ak09915_delayed_read_proc, this);
}

/*!
 * @brief Get dynamic range handler.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Dynamic range
 */
static UInt16 ak09915_get_dynamic_range(PhysicalSensorDescriptor *self)
{
    return this->maxDynamicRange;
}

/********************************************************************************/
/* descriptor of physical sensor */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor ak09915_descriptor =
{
    .device =
    {
        .options =
        {
            .i2c =
            {
                .hasClockStretching = 0,
                /* docs say 2.5MHz, but even 1000 is not reliable on Shuttles */
                .maxClock = AK09915_I2C_CLOCK,
            }, //lint !e785
        },

        .interface =
        {
            .options = InterfaceAnyI2C,
        },

        .irqEdge = IRQ_EDGE_RISING,
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = AK09915_RESOLUTION,
    }, //lint !e785

    .type =
    {
        .value = BSX_INPUT_ID_MAGNETICFIELD,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
    }, //lint !e785

    .initialize = ak09915_initialize,

    .set_power_mode = ak09915_set_power_mode,
    .get_power_mode = ak09915_get_power_mode,

    .set_sample_rate = ak09915_set_sample_rate,
    .get_sample_rate = ak09915_get_sample_rate,

    .enable_interrupts = ak09915_enable_interrupts,
    .disable_interrupts = ak09915_disable_interrupts,

    .get_scale_factor = ak09915_get_scale_factor,
    .get_sample_data = ak09915_get_sample_data,

    .set_dynamic_range = NULL,
    .get_dynamic_range = ak09915_get_dynamic_range,

    .sensorData = (void *)g_ak09915_sensor_data,
    /* +/-4912 uT */
    .maxDynamicRange = AK09915_MAX_RANGE,
    /* 100Hz */
    .maxRate = AK09915_MAX_ODR,
    /* 0.9mA @100Hz from data sheet */
    .maxCurrent = AK09915_MAX_CURRENT,
    .minRate = AK09915_MIN_ODR,
}; //lint !e785

/** @}*/
