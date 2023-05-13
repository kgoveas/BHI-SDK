/*!
 * Copyright (C) Robert Bosch. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
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
 * @file         VirtSignificantMotion.c
 *
 * @brief        Virtual driver for the significant motion sensor.
 *
 * @date        7 Sep 2018
 *
 */
/*!
 *@addtogroup bhy2_group
 *@brief
 *@{*/

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <host.h>
#include <arc.h>

/*********************************************************************/
/* local macro definition */
#define DRIVER_REV    (1u)

/*!
 * @brief Virtual sensor processing callback. This function is here to provide unique interface for virtual drivers 
 *
 * @param[in] self : Descriptor for virtual sensor
 * @param[in] data : Physical raw data pointer
 *
 * @return SensorOK for sucessful data report. Other values for failure.
 */
SensorStatus virtual_significant_motion_handle_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    UNUSED(data);
    PhysicalSensorDescriptor *phys_sig_motion_p = self->triggerSource.phys;
    /* check physical sensor exist or not */
    if (NULL == phys_sig_motion_p)
    {
        return SensorErrorNonExistant;
    }

    /* Report event to sensor framework */
    reportSensorEvent(self, NULL, self->timestamp);
    /* This is a oneshot sensor and therefore disabling itself after reporting event as Android define */
    updateRequestedRate(cast_VirtualToHeader(self), 0.0F);
    return SensorOK;
}

/*!
 * @brief This function reports the last sensor data called by sensor framework if data loss.
 *
 * @param[in] self : Descriptor for virtual sensor
 *
 * @return SensorOK for sucessful data report. Other values for failure.
 */
SensorStatus virtual_significant_motion_get_last_sensor_data(VirtualSensorDescriptor *self)
{
    PhysicalSensorDescriptor *phys_sig_motion_p = self->triggerSource.phys;
    /* check physical sensor exist or not */
    if (NULL == phys_sig_motion_p)
    {
        return SensorErrorNonExistant;
    }

    reportSensorEvent(self, NULL, self->timestamp);
    return SensorOK;
}

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virt_significant_motion_descriptor = {
    .triggerSource = {
        .sensor = {
            .type = {
                .value = SENSOR_TYPE_INPUT_SIGNIFICANT_MOTION,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource = {
        .sensor = {
            .type = {
                .value = SENSOR_TYPE_INPUT_SIGNIFICANT_MOTION,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .info = {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type = {
        .value = SENSOR_TYPE_WAKE_SIGNIFICANT_MOTION,
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
        .no_decimation = TRUE,
        .wakeup_ap = TRUE,
        .on_change = TRUE, /* this is an Android "one-shot" sensor, but on-change as far as lost events is concerned */
    }, //lint !e785

    .outputPacketSize = 0,
    .priority = PRIORITY_6, /* low priority */

    .initialize = NULL,
    .handle_sensor_data = virtual_significant_motion_handle_sensor_data,
    .get_last_sensor_data = virtual_significant_motion_get_last_sensor_data,
}; //lint !e785

/** @}*/
