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
  * @file            ex_camera.c
  *
  * @date            01/09/2017
  *
  * @brief           driver of External Camera shuttle
  *
  */
/*!
 * @addtogroup Ex_Camera_driver
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
#include "ex_camera.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV    (1u)
#define expansion     ((excamera_detect_data_t *)self->expansionData)

/*********************************************************************/
/* variables declarations */
excamera_detect_data_t excamera_expansion_data;

/*********************************************************************/
/* static variables definition */
static float excamera_sample_rate = 0;

/*********************************************************************/
/* global function declarations */
SensorStatus excamera_enable_interrupts(PhysicalSensorDescriptor *self);
SensorStatus excamera_disable_interrupts(PhysicalSensorDescriptor *self);

/*********************************************************************/
/*! @name Global Functions */

/*!
 * @brief Initializes the external camera sensor into a known state.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @retval SensorOK                    The sensor has already been initialized
 * @retval SensorErrorUnexpectedDevice The sensor did not return expected results.
 * @retval SensorUnknownError          An unknown error has occured.
 */
SensorStatus excamera_initialize_sensor(PhysicalSensorDescriptor *self)
{
    if (self == NULL)
    {
        return SensorErrorNonExistant;
    }

    memset((void *)&excamera_expansion_data, 0, sizeof(excamera_detect_data_t));

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
SensorPowerMode excamera_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    switch (mode)
    {
        case SensorPowerModePowerDown:
        case SensorPowerModeSuspend:
            (void)excamera_disable_interrupts(self);
            break;

        case SensorPowerModeActive:
        case SensorPowerModeLowPowerActive:
            (void)excamera_enable_interrupts(self);
            break;

        default:
            return expansion->powerMode;
    }

    expansion->powerMode = mode;

    sensorPowerModeChanged(SensorOK, self);

    return mode;
}

/*!
 * @brief Used to get dynamic range.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The current dynamic of the sensor.
 */
uint16_t excamera_get_dynamic_range(PhysicalSensorDescriptor *self)
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
SensorPowerMode excamera_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    excamera_detect_data_t *data = expansion;
    return data->powerMode;
}

/*!
 * @brief Get the current sample rate of the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns The current sample rate of the sensor.
 */
float excamera_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return excamera_sample_rate;
}

/*!
 * @brief Requests the hardware to perform sample conversions at the specified rate.
 *
 * @param[in] sample_rate : The requested sample rate of the sensor in Hz.
 * @param[in] self        : Current physical sensor descriptor
 *
 * @returns The actual sample rate of the sensor.
 */
float excamera_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    if (sample_rate > self->maxRate)
    {
        sample_rate = self->maxRate;
    }
    if (sample_rate < self->minRate)
    {
        sample_rate = self->minRate;
    }

    excamera_sample_rate = sample_rate;
    sensorRateChanged(SensorOK, self);

    return excamera_sample_rate;
}

/*!
 * @brief Enables the interrupt request from the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns Status of the sensor
 */
SensorStatus excamera_enable_interrupts(PhysicalSensorDescriptor *self)
{
    self->int_enabled = TRUE;
    return SensorOK;
}

/*!
 * @brief Disables the interrupt request from the sensor.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @returns Status of the sensor
 */
SensorStatus excamera_disable_interrupts(PhysicalSensorDescriptor *self)
{
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
float excamera_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return EXCAMERA_SCALE_FACTOR;
}

/*!
 * @brief Reports to the framework that all sample data has been read out.
 *
 * @param[in] self : Current physical sensor descriptor
 *
 */
void excamera_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor *self)
{
    callback((void *)self);
}

/*********************************************************************/
/* descriptor of physical sensor */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor excamera_descriptor =
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
        .value = SENSOR_TYPE_INPUT_EXCAMERA,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
        .no_hang = TRUE,
    },

    .initialize = excamera_initialize_sensor,

    .set_power_mode = excamera_set_power_mode,
    .get_power_mode = excamera_get_power_mode,

    .set_sample_rate = excamera_set_sample_rate,
    .get_sample_rate = excamera_get_sample_rate,

    .enable_interrupts = excamera_enable_interrupts,
    .disable_interrupts = excamera_disable_interrupts,

    .get_scale_factor = excamera_get_scale_factor,
    .get_sample_data = excamera_get_sample_data,

    .set_dynamic_range = NULL,
    .get_dynamic_range = excamera_get_dynamic_range,

    .expansionData = (void *) &excamera_expansion_data,

    .maxDynamicRange = EXCAMERA_DYNAMIC_MAX_RAGNE,
    .maxRate = EXCAMERA_RATE_MAX,
    .minRate = EXCAMERA_RATE_MIN,
    .maxCurrent = EXCAMERA_MAX_CURRENT,
};

/** @}*/
