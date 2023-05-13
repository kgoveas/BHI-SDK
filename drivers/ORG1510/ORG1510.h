/*
 * Copyright (c) Robert Bosch. All rights reserved. Confidential.
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
 * @file            ORG1510.h
 *
 * @date            12/20/2018
 *
 * @brief           ORG1510 GPS physical driver
 *
 */
/*!
 * @addtogroup ORG1510_GPS_driver
 * @brief
 * @{*/


#ifndef _ORG1510_H_
#define _ORG1510_H_

/*********************************************************************/
/* system header files */
#include <types.h>

/*********************************************************************/
/* macro definitions */
#define VIRDBG_MSG                  0
#if VIRDBG_MSG
#include <stdio.h>
#define DEBUG_PRINT(fmt, args...)   printf(fmt, ##args);
#else
#define DEBUG_PRINT(fmt, args...)
#endif


#define ORG1510_RATE_MAX       5
#define ORG1510_RATE_MIN       1
#define ORG1510_MAX_CURRENT    0.160F
#define ORG1510_DYNAMIC_RANGE  16
#define ORG1510_SCALE_FACTOR   0.0F

/*********************************************************************/
/* extern variable declarations */
/*! GPS on-off pin number */
extern UInt8 gGPSOnOffPin;
/*! GPS reset pin number */
extern UInt8 gGPSResetPin;
/*! GPS wakeup pin number */
extern UInt8 gGPSWkupPin;
/*! GPS 1PP pin number */
extern UInt8 gGPS1PPPin;

/*********************************************************************/
/* function prototype declarations */

/*!
 * @brief Delay for specific milliseconds
 *
 * @param[in] ms : millisecond.
 *
 */
void org1510_delay_ms(uint16_t ms);

/*!
 * @brief ORG1510 reset pin int
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_reset_pin_init(void);

/*!
 * @brief ORG1510 reset pin set low
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_reset_pin_low(void);

/*!
 * @brief ORG1510 reset pin set high
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_reset_pin_high(void);

/*!
 * @brief ORG1510 on off pin int
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_on_off_pin_init(void);

/*!
 * @brief ORG1510 on off pin excute one time.
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_on_off(void);

/*!
 * @brief ORG1510 wakeup pin int
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_wkup_pin_init(void);

/*!
 * @brief ORG1510 wakeup pin read
 *
 * @return wakeup pin status.
 */
bool org1510_wkup_pin_read(void);

/*!
 * @brief ORG1510 1pp pin int
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_1pp_pin_init(void);

/*!
 * @brief ORG1510 cs pin init
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_cs_pin_init(void);

/*!
 * @brief ORG1510 cs pin set low
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_cs_pin_low(void);

/*!
 * @brief ORG1510 cs pin set high
 *
 * @return SensorOK for success, other values for failure.
 */
SensorStatus org1510_cs_pin_high(void);

#endif
/** @}*/
