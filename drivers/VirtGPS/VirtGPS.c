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
  * @file            VirtGPS.c
  *
  * @date            08/16/2017
  *
  * @brief           virtual sensor driver of GPS
  *
  */
/*!
 * @addtogroup VirtGPS
 * @brief
 * @{*/

/*********************************************************************/
/* system header files */
#include <stdio.h>
#include <SensorAPI.h>
#include <host.h>
#include <arc.h>
#include <hw_versions.h>

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV           (1u)
#define GPS_TRANSFER_SIZE    (20)

/*********************************************************************/
/* struct declarations */
typedef struct VIRTUAL_GPS_PACKET
{
    uint16_t msg_length;
    uint16_t msg_transfered;
    uint8_t packet_size;
    uint8_t protocol_type;
    uint8_t packet[GPS_TRANSFER_SIZE];
} __attribute__ ((packed)) virtual_gps_packet_t;

/*!
* @brief Initialization function of GPS module
*
* @param[in] self : Current virtual sensor descriptor
*
* @return Result of operation
*/
SensorStatus virt_gps_initialize(VirtualSensorDescriptor *self)
{
    UNUSED(self);
    return SensorOK;
}

/*!
* @brief Report GPS messages to host
*
* @param[in] self : Current virtual sensor descriptor
* @param[in] data : Physical raw data
*
* @return Result of operation
*/
SensorStatus virt_gps_handle_data(VirtualSensorDescriptor *self, void *data)
{
    virtual_gps_packet_t virt_gps_packet;
    virtual_gps_packet_t *p_phy_gps_packet;
    PhysicalSensorDescriptor *p_phy_gps = self->triggerSource.phys;

    if (NULL == p_phy_gps)
    {
        return SensorDataUnavailable;
    }

    p_phy_gps_packet = (virtual_gps_packet_t *)data;
    /* get message length */
    virt_gps_packet.msg_length = p_phy_gps_packet->msg_length;
    /* get transfered length */
    virt_gps_packet.msg_transfered = p_phy_gps_packet->msg_transfered;
    /* get current packet size */
    virt_gps_packet.packet_size = p_phy_gps_packet->packet_size;
    /* get protocol type */
    virt_gps_packet.protocol_type = p_phy_gps_packet->protocol_type;
    /* get payload */
    memset(virt_gps_packet.packet, 0, sizeof(virt_gps_packet.packet));
    memcpy(virt_gps_packet.packet, p_phy_gps_packet->packet, sizeof(virt_gps_packet.packet));

    if (virt_gps_packet.msg_length)
    {
        reportSensorEvent(self, &virt_gps_packet, self->timestamp);
    }
    return SensorOK;
} //lint !e818


/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtgps_descriptor =
{
    .triggerSource =
    {
        .sensor = {
            .type = {
                .value = SENSOR_TYPE_INPUT_GPS,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            },//lint !e785
        },
    },

    .physicalSource =
    {
        .sensor = {
            .type = {
                .value = SENSOR_TYPE_INPUT_GPS,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            },//lint !e785
        },
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    },//lint !e785

    .type =
    {
        .value = SENSOR_TYPE_GPS,
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    },//lint !e785

    .outputPacketSize = sizeof(virtual_gps_packet_t),
    .priority = PRIORITY_4, /* medium priority */

    .initialize = virt_gps_initialize,
    .handle_sensor_data = virt_gps_handle_data,
}; //lint !e785

/** @}*/

