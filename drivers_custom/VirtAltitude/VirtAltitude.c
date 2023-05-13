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
 * @file            VirtAltitude.c
 *
 * @date          12/20/2018
 *
 * @brief          Virtual Driver of Any Motion Sensor
 *
 */

/*!
 *@addtogroup VirtAltitude
 *@brief
 *@{*/

/*********************************************************************/
/* system header files */
#include <I2CDriverInterface.h>
#include <SensorAPI.h>
#include <host.h>
#include <math.h>

/* altitude formula */
/* P = P0 * (1-H/44300)^5.256 , P0 = pressure of 0 deg (101.325 kPa) */

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV                              (4u)
#define SENSOR_TYPE_ALTITUDE_SENSOR             (SENSOR_TYPE_CUSTOMER_VISIBLE_START + 1)
#define PARAM_PAGE_OPTIONAL_SDK                 (8)
#define OPTIONAL_SDK_PARAM_ALTITUDE_SEA_LEVEL   (0x00)
#define DEFAULT_REFERENCE_PRESSUE_SEA_LEVEL      101325.0F

/*********************************************************************/
/* structure definition */
typedef struct
{
    int32_t     altitude;
} __attribute__ ((packed)) output_t;

/*********************************************************************/
/* local variables */
static float reference_pressure_sea_level = DEFAULT_REFERENCE_PRESSUE_SEA_LEVEL;

/*!
 * @brief optional sdk for parameter page write
 *
 * @param[in] param       : parameter index
 * @param[in] length      : write length
 * @param[in] buffer      : write buffer payload
 *
 * @return Execution result
 */
bool optional_sdk_page_write_handler(uint8_t param, uint16_t length, uint8_t buffer[])
{
    switch (param)
    {
        case OPTIONAL_SDK_PARAM_ALTITUDE_SEA_LEVEL:
        {
            uint32_t reference_pressure_temp = *(uint32_t*)buffer;
            reference_pressure_sea_level = (float)reference_pressure_temp / 10.0F;
            return TRUE;
        }
        default:
            break;
    }
    return FALSE;
}

/*!
 * @brief optional sdk for parameter page read
 *
 * @param[in] param       : parameter index
 * @param[in] length      : read length
 * @param[in] buffer      : read buffer payload ret_length
 * @param[out] ret_length : actual read length already
 *
 * @return Execution result
 */
bool optional_sdk_page_read_handler(uint8_t param, uint16_t length, uint8_t buffer[], uint16_t *ret_length)
{
    switch (param)
    {
        case OPTIONAL_SDK_PARAM_ALTITUDE_SEA_LEVEL:
        {
            uint32_t *reference_pressure_temp = (uint32_t *)buffer;
            *reference_pressure_temp = (uint32_t)(reference_pressure_sea_level * 10.0F);
            *ret_length = 4;
            return TRUE;
        }
        default:
            break;
    }
    return FALSE;
}

/*!
 * @brief Handle altitude sensor data
 *
 * @param[in] self       : Current virtual sensor descriptor
 * @param[out] data    : Sensor data
 *
 * @return Execution result
 */
static SensorStatus virt_altitude_handle_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    PhysicalSensorDescriptor* phy_bme = self->triggerSource.phys;
    float pressure_f;
    float altitude_in_meters = 0.0F;
    output_t output;
    uint32_t comp_p;

    UNUSED(data);

    comp_p = *((uint32_t *)phy_bme->sensorData);
    pressure_f = (float)((float)(comp_p * 100.0F / 128) / reference_pressure_sea_level); /* 128LSB/Pa */
    altitude_in_meters = (float)(44300.0F * (1.0F - powf(pressure_f, 0.190294F))); /* altitude formula */
    output.altitude = (SInt32)(altitude_in_meters * 100.0F);

    reportSensorEvent((const VirtualSensorDescriptor *)self, &output, self->timestamp);
    return SensorOK;
} //lint !e818

/*!
 * @brief Initialize altitude sensor data
 *
 * @param[in] self       : Current virtual sensor descriptor
 *
 * @return Execution result
 */
static SensorStatus virt_altitude_initialize_sensor(VirtualSensorDescriptor *self)
{
    (void) registerWriteParamHandler(PARAM_PAGE_OPTIONAL_SDK, optional_sdk_page_write_handler);
    (void) registerReadParamHandler(PARAM_PAGE_OPTIONAL_SDK, optional_sdk_page_read_handler);
    return SensorOK;
}

/*********************************************************************/
/* descriptor of virtual sensor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virt_altitude_descriptor =
{
    .triggerSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_PRESSURE,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_PRESSURE,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type =
    {
        .value = SENSOR_TYPE_ALTITUDE_SENSOR,
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .outputPacketSize = sizeof(output_t),
    .priority = PRIORITY_6,    /* Low priority */
    .initialize = virt_altitude_initialize_sensor,
    .handle_sensor_data = virt_altitude_handle_sensor_data,
}; //lint !e785

/** @}*/
