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
 * @file            TMG4903ProximitySensor.c
 *
 * @date          12/20/2018
 *
 * @brief          Physical sensor of TMG4903 proximity
 *
 */

/*!
 *@addtogroup TMG4903 physical driver
 *@brief
 *@{*/

/*********************************************************************/
/* system header files */
#include <I2CDriverInterface.h>
#include <Timer.h>
#include <arc.h>
#include <host.h>

/*********************************************************************/
/* local header files */
#include "../TMG4903Light/TMG4903LightSensor.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV      (1u)

/*********************************************************************/
/* global variables */
static float tmg4903_proximity_sample_rate;
static SensorPowerMode tmg4903_proximity_power_mode = SensorPowerModePowerDown;
static uint8_t proximity_report = TMG4903_PROXIMITY_CLOSE;

/*********************************************************************/
/* functions */

/* Allow deprecated API calls from the initialize function. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/*!
 * @brief Initiate tmg4903 proximity driver
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 *    SensorOK                       The sensor has already been initialized
 *    SensorErrorUnexpectedDevice    The sensor did not return expected results.
 *    SensorErrorNonExistant         No device
 */
SensorStatus tmg4903_proximity_initialize_sensor(PhysicalSensorDescriptor *self)
{
    SensorStatus return_val = tmg4903_initialize_sensor(self);

    if (return_val == SUCCESS)
    {
        proximity_report = TMG4903_PROXIMITY_INIT;
    }
    return return_val;
}

#pragma GCC diagnostic pop

/*!
 * @brief Do self-test for TMG4903 proximity sensor
 *
 * @param[in] device : Current physical sensor
 *
 * @return Self-test result
 */
SensorStatus tmg4903_proximity_selftest(const PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    /* add self test code here */
    return SensorSelfTestPassed;
}

/*!
 * @brief Set sample rate for TMG4903 proximity sensor
 *
 * @param[in] sample_rate : Sample rate to be set
 * @param[in] self        : Current physical sensor descriptor
 *
 * @return Sample rate
 */
static float tmg4903_proximity_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    float maxrate = self->maxRate;

    if (sample_rate > maxrate)
    {
        sample_rate = maxrate;
    }

    tmg4903_proximity_sample_rate = sample_rate;
    sensorRateChanged(SensorOK, self);

    return tmg4903_proximity_sample_rate;
}

/*!
 * @brief Get sample rate for TMG4903 proximity sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Sample rate
 */
static float tmg4903_proximity_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return tmg4903_proximity_sample_rate;
}

/*!
 * @brief Get scale facotr for TMG4903 proximity sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Scale factor
 */
static float tmg4903_proximity_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    /* The scale factor from the datasheet to convert from the uint32_t result to floating point (in uT)*/
    return TMG4903_SCALE_FACTOR;
}

/*!
 * @brief Set range for TMG4903 proximity sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Range
 */
static uint16_t tmg4903_proximity_get_dynamic_range(PhysicalSensorDescriptor *self)
{
    /* This is the dynamic range according to the datasheet.*/
    return self->maxDynamicRange;
}

/*!
 * @brief Set power mode for TMG4903 proximity sensor
 *
 * @param[in] mode : Power mode to be set
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Power mode
 */
static SensorPowerMode tmg4903_proximity_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    bool update_flag = FALSE;
    bool enable_flag = FALSE;

    if (mode != tmg4903_proximity_power_mode)
    {
        switch (mode)
        {
            case SensorPowerModePowerDown:
            case SensorPowerModeSuspend:
                update_flag = TRUE;
                enable_flag = FALSE;
                tmg4903_reg_data_raw.proximity_raw = 0;
                proximity_report = TMG4903_PROXIMITY_INIT;
                break;

            case SensorPowerModeLowPowerActive:
            case SensorPowerModeActive:
                update_flag = TRUE;
                enable_flag = TRUE;
                break;

            case SensorPowerModeFOC:
            {
                /* offset */
                int16_t offsets[] = { 0, 0, 0 };
                SensorStatus tmg4903_status = SensorErrorFOCFailed;

                /* add foc subroutine here */
                reportFOCResults((SensorDescriptorHeader*)(self), tmg4903_status, offsets[0], offsets[1], offsets[2]);
                reportSensorStatus((SensorDescriptorHeader*)(self), tmg4903_status);
                break;
            }

            default:
                return tmg4903_proximity_power_mode;
        }
    }

    if (update_flag)
    {
        /* update device power mode */
        (void)tmg4903_device_set_power_mode(mode, self);

        /* enable interrupt , turn on interrupt switch */
        if (enable_flag)
        {
            tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                                  1u << TMG4903_ENABLE_PROXIMITY_INTERRUPT | 1u << TMG4903_ENABLE_PROXIMITY, TRUE);
        }

        tmg4903_proximity_power_mode = mode;
    }
    sensorPowerModeChanged(SensorOK, self);

    return tmg4903_proximity_power_mode;
}

/*!
 * @brief Get power mode for TMG4903 proximity sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Power mode
 */
static SensorPowerMode tmg4903_proximity_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return tmg4903_proximity_power_mode;
}

/*!
 * @brief Get sample data of TMG4903 proximity sensor
 *
 * @param[in] callback : Callback function
 * @param[in] self     : Current physical sensor descriptor
 *
 * @ return None
 */
static void tmg4903_proximity_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor *self)
{
    if (tmg4903_reg_data_raw.proximity_raw > 0)
    {

        /* the value >= TMG4903_PROXIMITY_THRES_HIGH */
        if (tmg4903_reg_data_raw.proximity_raw >= TMG4903_PROXIMITY_THRES_HIGH)
        {
            if (proximity_report != TMG4903_PROXIMITY_CLOSE)
            {
                proximity_report = TMG4903_PROXIMITY_CLOSE;
                callback((SensorDescriptorHeader*)self);
            }
        }
        else/* the value <= TMG4903_PROXIMITY_THRES_LOW */
        {
            if (proximity_report != TMG4903_PROXIMITY_AWAY)
            {
                proximity_report = TMG4903_PROXIMITY_AWAY;
                callback((SensorDescriptorHeader*)self);
            }
        }
    }
}

/*!
 * @brief Enable TMG4903 proximity sensor interrupt
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
static SensorStatus tmg4903_proximity_enable_interrupts(PhysicalSensorDescriptor *self)
{
    /* turn on interrupt switch */
    tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                          1u << TMG4903_ENABLE_PROXIMITY_INTERRUPT | 1u << TMG4903_ENABLE_PROXIMITY, TRUE);
    return tmg4903_origin_interrupts_op(TRUE, self);
}

/*!
 * @brief Disable TMG4903 proximity sensor interrupt
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
static SensorStatus tmg4903_proximity_disable_interrupts(PhysicalSensorDescriptor *self)
{
    /* turn on interrupt switch */
    tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                          1u << TMG4903_ENABLE_PROXIMITY_INTERRUPT | 1u << TMG4903_ENABLE_PROXIMITY, FALSE);

    return tmg4903_origin_interrupts_op(FALSE, self);
}

/********************************************************************************/
/* descriptor of physical sensor */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor tmg4903_proximity_descriptor =
{
    .device =
    {
        .options =
        {
            .i2c =
            {
                .hasClockStretching = 0,
                .maxClock = 400, /* 400kHz */
            },//lint !e785
        },//lint !e785

        .interface =
        {
            .options = InterfaceAnyI2C,
        },

        .irqEdge = IRQ_EDGE_FALLING,
    },//lint !e785

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = 16,
    },//lint !e785

    .type =
    {
        .value = SENSOR_TYPE_INPUT_PROXIMITY,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
        .no_hang = TRUE,
    },//lint !e785

    .initialize = tmg4903_proximity_initialize_sensor,

    .set_power_mode = tmg4903_proximity_set_power_mode,
    .get_power_mode = tmg4903_proximity_get_power_mode,

    .set_sample_rate = tmg4903_proximity_set_sample_rate,
    .get_sample_rate = tmg4903_proximity_get_sample_rate,

    .enable_interrupts = tmg4903_proximity_enable_interrupts,
    .disable_interrupts = tmg4903_proximity_disable_interrupts,

    .get_scale_factor = tmg4903_proximity_get_scale_factor,
    .get_sample_data = tmg4903_proximity_get_sample_data,

    .set_dynamic_range = NULL,
    .get_dynamic_range = tmg4903_proximity_get_dynamic_range,

    .sensorData = (void*) &proximity_report,

    .maxDynamicRange = TMG4903_DYNAMIC_RANGE_MAX,
    .maxRate = TMG4903_RATE_MAX,   /* 1 Hz */
    .minRate = TMG4903_RATE_MIN,   /* 1 Hz */
    .maxCurrent = TMG4903_CURRENT, /* 0.13 mA */
};//lint !e785

/** @}*/
