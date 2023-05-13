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
 * @file        ORG1510.c
 *
 * @brief       ORG1510 GPS physical driver
 *
 * @date        12/08/2018
 *
 */
/*!
 * @addtogroup ORG1510_GPS_driver
 * @brief
 * @{*/

/*********************************************************************/
/* system header files */
#include <arc.h>
#include <host.h>
#include <stdio.h>
#include <Timer.h>

/*********************************************************************/
/* local header files */
#include "ORG1510.h"

/* this needs to be configurable -- is done in ORG1510PhysicalGPS.c in the init function
 * MFPAD[1] on ASIC, MFPAD[7] on FPGA
 */
/*! GPS on-off pin number */
UInt8 gGPSOnOffPin;
/*! GPS reset pin number */
UInt8 gGPSResetPin;
/*! GPS wakeup pin number */
UInt8 gGPSWkupPin;
/*! GPS 1PP pin number */
UInt8 gGPS1PPPin;

/* Allow deprecated API calls from the initialize function. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/*!
 * @brief Delay for specific milliseconds
 */
void org1510_delay_ms(uint16_t ms)
{
    (void)delayums(ms);
}

#pragma GCC diagnostic pop

/*!
 * @brief ORG1510 reset pin int
 */
SensorStatus org1510_reset_pin_init(void)
{
    enableGPIOOutput(gGPSResetPin);
    setGPIOLow(gGPSResetPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 reset pin set low
 */
SensorStatus org1510_reset_pin_low(void)
{
    setGPIOLow(gGPSResetPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 reset pin set high
 */
SensorStatus org1510_reset_pin_high(void)
{
    setGPIOHigh(gGPSResetPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 wakeup pin int
 */
SensorStatus org1510_wkup_pin_init(void)
{
    disableGPIOOutput(gGPSWkupPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 wakeup pin read
 */
bool org1510_wkup_pin_read(void)
{
    return getGPIOValue(gGPSWkupPin);
}

/*!
 * @brief ORG1510 1pp pin int
 */
SensorStatus org1510_1pp_pin_init(void)
{
    disableGPIOOutput(gGPS1PPPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 on off pin int
 */
SensorStatus org1510_on_off_pin_init(void)
{
    enableGPIOOutput(gGPSOnOffPin);
    disableGPIOPull(gGPSOnOffPin);
    setGPIOLow(gGPSOnOffPin);
    return SensorOK;
}

/*!
 * @brief ORG1510 cs pin init
 */
SensorStatus org1510_cs_pin_init(void)
{
    return SensorOK;
}

/*!
 * @brief ORG1510 cs pin set low
 */
SensorStatus org1510_cs_pin_low(void)
{
    return SensorOK;
}

/*!
 * @brief ORG1510 cs pin set high
 */
SensorStatus org1510_cs_pin_high(void)
{
    return SensorOK;
}

/*!
 * @brief ORG1510 on off pin excute one time.
 */
SensorStatus org1510_on_off(void)
{
    /* generate an ON_OFF pulse */
    setGPIOLow(gGPSOnOffPin);
    org1510_delay_ms(1);
    setGPIOHigh(gGPSOnOffPin);
    org1510_delay_ms(5);
    setGPIOLow(gGPSOnOffPin);
    org1510_delay_ms(5);
    return SensorOK;
}

/** @}*/

