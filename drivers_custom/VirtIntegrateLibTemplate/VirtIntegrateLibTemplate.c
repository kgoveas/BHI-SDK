/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 * @file            VirtIntegrateLibTemplate.c
 *
 * @date            09/06/2018
 *
 * @brief           Template driver for integrated library
 *
 *
 */

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <host.h>
#include <stdio.h>
#include <arc.h>
#include <hw_versions.h>
#include "lib_template.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV             (1u)
#define SENSOR_INPUT           BSX_INPUT_ID_ACCELERATION            /* Physical sensor to use for trigger source (source data) */
#define SENSOR_TYPE_TEMPLATE   (SENSOR_TYPE_CUSTOMER_VISIBLE_START + 2)

/*********************************************************************/
/* structure definition */
typedef struct
{
    bool     result;
} __attribute__ ((packed)) output_t;

/*********************************************************************/
/* function definitions */

/*!
 * @brief This function reports the return value from libtemplate
 *
 * @param[in] self : Current virtual sensor descriptor
 * @param[in] data : Physical raw data
 *
 * @return Result of execution
 */
static SensorStatus handle_sensor_data(VirtualSensorDescriptor* self, void* data)
{
    output_t output;
    PhysicalSensorDescriptor* parent = cast_HeaderToPhysical(getSensorParent(cast_VirtualToHeader(self)));
    float dynamicRange = getDynamicRange(cast_PhysicalToHeader(parent));
    float scaleFactor = parent->get_scale_factor(parent) * (float)MAX_SINT16 / dynamicRange; /* Scale to dynamic range, 16bit signed output */
    SystemTime_t timestamp = self->timestamp;
    int32_t *sendata = data;
    int32_t xi, yi, zi;
    float x, y, z;
    float acceleration_p[3];

    xi = sendata[0];
    yi = sendata[1];
    zi = sendata[2];

    x = xi * scaleFactor;
    y = yi * scaleFactor;
    z = zi * scaleFactor;

    acceleration_p[0] = SATURATE(MAX_SINT16, x, MIN_SINT16);
    acceleration_p[1] = SATURATE(MAX_SINT16, y, MIN_SINT16);
    acceleration_p[2] = SATURATE(MAX_SINT16, z, MIN_SINT16);

    /* call the integrated lib API if needed , the API maybe use the physical data as input, maybe use the BSX output as input. 
    here our example just use physical data as its input */
    bool result = lib_template_api(acceleration_p);

    output.result = result;

    reportSensorEvent(self, &output, timestamp);

    return SensorOK;
} //lint !e818

VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virt_integrate_lib_template_descriptor = {
    .triggerSource = {
        .sensor = {
            .type = {
                .value = SENSOR_INPUT,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource = {
        .sensor = {
            .type = {
                .value = SENSOR_INPUT,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        }
    },

    .info = {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type = {
        .value = SENSOR_TYPE_TEMPLATE,
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
        .wakeup_ap = FALSE,
    }, //lint !e785

    .outputPacketSize = sizeof(output_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = handle_sensor_data,
}; //lint !e785
