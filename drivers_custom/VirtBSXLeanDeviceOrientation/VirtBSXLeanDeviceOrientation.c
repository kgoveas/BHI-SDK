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
  * @file            VirtBSXLeanDeviceOrientation.c
  *
  * @date            09/06/2018
  *
  * @brief           Calculate lean device orientation data from accelerometer data source.
  *                  This is a demo driver to demonstrate how to use the custom output from BSX.
  *                  This driver depends on VirtBSXCustomAccelDataSource driver.
  *                  This sensor prints device orientation continuously.
  *                  The device orientation in this driver is defined as:
  *                      - X+: X axis of device is pointing to the sky
  *                      - X-: X axis of device is pointing to the earth
  *                      - Y+: Y axis of device is pointing to the sky
  *                      - Y-: Y axis of device is pointing to the earth
  *                      - Z+: Z axis of device is pointing to the sky
  *                      - Z-: Z axis of device is pointing to the earth
  *                      - NA: Device orientation not available
  *                   The output from this demo is the 2-byte ascii code for "X+", "X-", "Y+", "Y-", "Z+", "Z-"
  */

#include <SensorAPI.h>
#include <host.h>
#include <arc.h>
#include <hw_versions.h>
#include <bsx_support.h>

#define DRIVER_REV      (1u)
#define SENSOR_TYPE_BSX_LEAN_DEVICE_ORIENTATION   SENSOR_TYPE_CUSTOMER_VISIBLE_START

#define DEBUG_DRIVER    1
#if DEBUG_DRIVER
#include <stdio.h>
#define verbose(...)    printf(__VA_ARGS__)
#else
#define verbose(...)
#endif

/*****************************/
/*        Global Variables   */
/*****************************/
typedef struct {
    UInt8      orientation[2];
} __attribute__ ((packed)) output_t;
static output_t g_output;

/*!
 * @brief Initiate
 *
 * @param[in] self : Current virtual sensor descriptor
 *
 * @return Status
 */
static SensorStatus ldo_initialize(VirtualSensorDescriptor *self)
{
    verbose("LDO initialize\n");
    UNUSED(self);
    g_output.orientation[0] = 'N';
    g_output.orientation[1] = 'A';
    return SensorOK;
}

/*!
 * @brief Change power mode
 *
 * @param[in] self : Current virtual sensor descriptor
 * @param[in] mode : Power mode
 *
 * @return None
 */
static void ldo_on_power_mode_changed(VirtualSensorDescriptor *self, SensorPowerMode mode)
{
    verbose("LDO power mode changed\n");
    UNUSED(mode);
    UNUSED(self);
    g_output.orientation[0] = 'N';
    g_output.orientation[1] = 'A';
}

/*!
 * @brief Handle sensor data
 *
 * @param[in] self  : Current virtual sensor descriptor
 * @param[in] data  : Sensor data
 *
 * @return None
 */
static SensorStatus ldo_handle_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    const bsx_fifo_data_t* bsxdata = data;
    UInt8 i;
    float tmp = 0.0F;

    g_output.orientation[0] = 'N';
    g_output.orientation[1] = 'A';

    for (i = 0; i < 3; ++i)
    {
        if (bsxdata->content_p[i].sfp > tmp)
        {
            tmp = bsxdata->content_p[i].sfp;
            g_output.orientation[0] = 'X' + i;
            g_output.orientation[1] = '+';
        }
        else if (bsxdata->content_p[i].sfp < -tmp)
        {
            tmp = -bsxdata->content_p[i].sfp;
            g_output.orientation[0] = 'X' + i;
            g_output.orientation[1] = '-';
        }
    }

    verbose("ldo %d %d\n", g_output.orientation[0], g_output.orientation[1]);
    reportSensorEvent(self, &g_output, self->timestamp);

    return SensorOK;
} //lint !e818

/*********************************************************************/
/* virtual sensor descripor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor descriptor_virt_bsx_lean_device_orientation = {
    .triggerSource = {
        .sensor = {
            .type = {
                .value = SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ACCELERATION_CORRECTED),
                .flags = DRIVER_TYPE_VIRTUAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource = {
        .sensor = {
            .type = {
                .value = BSX_INPUT_ID_ACCELERATION,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .info = {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type = {
        .value = SENSOR_TYPE_BSX_LEAN_DEVICE_ORIENTATION,
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .maxRate = 800.0F,
    .minRate = 1.5625F,

    .outputPacketSize = sizeof(output_t),
    .priority = PRIORITY_2, // high priority

    .initialize = ldo_initialize,
    .handle_sensor_data = ldo_handle_sensor_data,
    .mode_changed = ldo_on_power_mode_changed,
}; //lint !e785
