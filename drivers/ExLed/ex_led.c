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
  * @file            ex_led.c
  *
  * @date            10/08/2020
  *
  * @brief           driver of External led in shuttle
  *
  */
/*!
 * @addtogroup Ex_Led_driver
 * @brief
 * @{*/

/*********************************************************************/
/* system header files */
#include <I2CDriverInterface.h>
#include <Timer.h>
#include <stdio.h>
#include <string.h>
#include <arc.h>
#include <host.h>
#include <SensorAPI.h>

/*********************************************************************/
/* local header files */
#include "ex_led.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV (1u)

/*********************************************************************/
/* variables declarations */

/*********************************************************************/
/* static variables definition */
static float exled_sample_rate = 0;
static SInt8 g_exled_timer_index = -1;
static UInt8 g_LedPin = 0;
static UInt8 led_status = 0;
static SensorPowerMode g_led_sensor_mode;

/*********************************************************************/
/* global function declarations */
SensorStatus exled_enable_interrupts(PhysicalSensorDescriptor *self);
SensorStatus exled_disable_interrupts(PhysicalSensorDescriptor *self);

/*********************************************************************/
/*! @name Global Functions */

/*!
 * @brief This function trigger sensor to read out raw data.
 *
 * @param[in] status : the return value of callback function
 * @param[in] sensor : the descriptor of sensor
 *
 * @return Result of execution
 */
static void exled_event_trigger_read(SensorStatus status, void *sensor)
{
    UNUSED(status);
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)sensor;
    if (led_status == 0)
    {
        setGPIOLow(g_LedPin);
        led_status = 1;
    }
    else if (led_status == 1)
    {
        setGPIOHigh(g_LedPin);
        led_status = 0;
    }
    sensorInterruptHandler(getSystemTime(), self);
}

/*!
 * @brief Initializes the external led sensor into a known state.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @retval SensorOK                    The sensor has already been initialized
 * @retval SensorErrorUnexpectedDevice The sensor did not return expected results.
 * @retval SensorUnknownError          An unknown error has occured.
 */
SensorStatus exled_initialize_sensor(PhysicalSensorDescriptor *self)
{
    if (self == NULL)
    {
        return SensorErrorNonExistant;
    }

    if (g_exled_timer_index < 0)
    {
        /* this function can be called multiple times, if we were already called, don't schedule another timer. */
        g_exled_timer_index = timer_schedule_polling(EXLED_DEFAULT_TIMER_ODR, exled_event_trigger_read, self);
    }
    g_LedPin = self->CalMatrix[0];

    enableGPIOOutput(g_LedPin);
    setGPIOHigh(g_LedPin);

    /* enter suspend mode to reset registers. */
    g_led_sensor_mode = SensorPowerModePowerDown;

    return SensorOK;
}

/*!
 * @brief If possible, sets the sensor to the requested power mode.
 *
 * ex: If the LowPowerActive mode was request but not supported, Active shall have been entered.
 *
 * @param[in] mode : The requested sensor mode.
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The actual state the sensor was set to.
 */
SensorPowerMode exled_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    switch (mode)
    {
    case SensorPowerModeSelfTest:
        reportSelfTestResults(cast_PhysicalToHeader(self), SensorOK, 0, 0, 0);
        g_led_sensor_mode = SensorPowerModePowerDown;
        sensorPowerModeChanged(SensorOK, self);
        return SensorPowerModeSelfTest;

    case SensorPowerModePowerDown:
    case SensorPowerModeSuspend:
        timer_disable_interrupt(g_exled_timer_index);
        g_led_sensor_mode = mode;
        break;

    case SensorPowerModeActive:
    case SensorPowerModeLowPowerActive:
        //(void)exled_enable_interrupts(self);
        g_led_sensor_mode = SensorPowerModeActive;
        break;

    default:
        return g_led_sensor_mode;
    }

    sensorPowerModeChanged(SensorOK, self);

    return g_led_sensor_mode;
}

/*!
 * @brief Used to get dynamic range.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The current dynamic of the sensor.
 */
uint16_t exled_get_dynamic_range(PhysicalSensorDescriptor *self)
{
    return self->maxDynamicRange; /* 2^16 -1, 1 count = 1 step */
}

/*!
 * @brief Get the current power mode of the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The current power mode of the sensor.
 */
SensorPowerMode exled_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return g_led_sensor_mode;
}

/*!
 * @brief Get the current sample rate of the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The current sample rate of the sensor.
 */
float exled_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return exled_sample_rate;
}

/*!
 * @brief Requests the hardware to perform sample conversions at the specified rate.
 *
 * @param[in] sample_rate : The requested sample rate of the sensor in Hz.
 * @param[in] self        : Current physical sensor descriptor
 *
 * @returns The actual sample rate of the sensor.
 */
float exled_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    if (sample_rate > self->maxRate)
    {
        sample_rate = self->maxRate;
    }
    if (sample_rate < self->minRate)
    {
        sample_rate = self->minRate;
    }

    exled_sample_rate = sample_rate;
    (void)timer_update_polling(g_exled_timer_index, exled_sample_rate);

    sensorRateChanged(SensorOK, self);

    return exled_sample_rate;
}

/*!
 * @brief Enables the interrupt request from the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns Status of the sensor
 */
SensorStatus exled_enable_interrupts(PhysicalSensorDescriptor *self)
{
    self->int_enabled = TRUE;
    timer_enable_interrupt(g_exled_timer_index);

    return SensorOK;
}

/*!
 * @brief Disables the interrupt request from the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns Status of the sensor
 */
SensorStatus exled_disable_interrupts(PhysicalSensorDescriptor *self)
{

    setGPIOHigh(g_LedPin);
    timer_disable_interrupt(g_exled_timer_index);
    self->int_enabled = FALSE;

    return SensorOK;
}

/*!
 * @brief Get the scale factor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns scal_factor
 */
float exled_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return EXLED_SCALE_FACTOR;
}

/*!
 * @brief Reports to the framework that all sample data has been read out.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 */
void exled_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor *self)
{
    callback((void *)self);
}

/*********************************************************************/
/* descriptor of physical sensor */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor exled_descriptor =
    {
        .device =
            {
                .options =
                    {
                        .i2c =
                            {
                                .hasClockStretching = 0,
                                .maxClock = 400, /* 400KHz */
                            },

                        .spi =
                            {
                                .maxClock = 10000, /* 10MHz */
                                .cpol = 1,
                                .cpha = 1,
                                .reg_shift = 0,
                                .read_pol = 1,
                                .read_bit = 7,
                            },
                    },

                .interface =
                    {
                        .options = InterfaceNone,
                    },

                .irqEdge = 1,
            },

        .info =
            {
                .id = DRIVER_ID,
                .version = DRIVER_REV,
                .resolution = 16,
            },

        .type =
            {
                .value = SENSOR_TYPE_INPUT_EXLED,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
                .no_hang = TRUE,
            },

        .initialize = exled_initialize_sensor,

        .set_power_mode = exled_set_power_mode,
        .get_power_mode = exled_get_power_mode,

        .set_sample_rate = exled_set_sample_rate,
        .get_sample_rate = exled_get_sample_rate,

        .enable_interrupts = exled_enable_interrupts,
        .disable_interrupts = exled_disable_interrupts,

        .get_scale_factor = exled_get_scale_factor,
        .get_sample_data = exled_get_sample_data,

        .set_dynamic_range = NULL,
        .get_dynamic_range = exled_get_dynamic_range,
        .sensorData = (void *)&led_status,

        .maxDynamicRange = EXLED_DYNAMIC_MAX_RANGE,
        .maxRate = EXLED_RATE_MAX,
        .minRate = EXLED_RATE_MIN,
        .maxCurrent = EXLED_MAX_CURRENT,
};

/** @}*/
