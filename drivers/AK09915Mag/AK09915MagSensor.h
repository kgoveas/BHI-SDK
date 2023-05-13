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
 * @file        AK09915MagSensor.h
 *
 * @date      12/20/2018
 *
 * @brief      Head for driver of AK09915 mag
 *
 */

/*!
 *@addtogroup ak09915_mag_group
 *@brief
 *@{*/

#ifndef AK09915MAGSENSOR_H
#define AK09915MAGSENSOR_H

#define AK09915_REG_WIA1                        (0x00)
#define AK09915_REG_WIA2                        (0x01)
#define AK09915_REG_RSV1                        (0x02)
#define AK09915_REG_RSV2                        (0x03)
#define AK09915_REG_ST1                         (0x10)
#define AK09915_REG_HXL                         (0x11)
#define AK09915_REG_HXH                         (0x12)
#define AK09915_REG_HYL                         (0x13)
#define AK09915_REG_HYH                         (0x14)
#define AK09915_REG_HZL                         (0x15)
#define AK09915_REG_HZH                         (0x16)
#define AK09915_REG_TMPS                        (0x17)
#define AK09915_REG_ST2                         (0x18)
#define AK09915_REG_CNTL1                       (0x30)
#define AK09915_REG_CNTL2                       (0x31)
#define AK09915_REG_CNTL3                       (0x32)
#define AK09915_REG_TS1                         (0x33)
#define AK09915_REG_TS2                         (0x34)
#define AK09915_REG_TS3                         (0x35)
#define AK09915_REG_I2CDIS                      (0x36)
#define AK09915_REG_TS4                         (0x37)
#define AK09915_REG_ASAX                        (0x60)
#define AK09915_REG_ASAY                        (0x61)
#define AK09915_REG_ASAZ                        (0x62)
#define AK09915_REG_TPH1                        (0xC0)
#define AK09915_REG_TPH2                        (0xC1)
#define AK09915_REG_RR                          (0xC2)
#define AK09915_REG_SYT                         (0xC3)
#define AK09915_REG_DT                          (0xC4)

#define AK09915_COMPANY_ID                      (0x48)
#define AK09915_DEVICE_ID                       (0x10)

#define AK09915_ST1_BIT_DRDY                    (1 << 0)
#define AK09915_ST1_BIT_DOR                     (1 << 1)
#define AK09915_ST1_BIT_HSM                     (1 << 7)

#define AK09915_ST2_BIT_HOFL                    (1 << 3)
#define AK09915_ST2_BIT_INV                     (1 << 2)

#define AK09915_CNTL1_NSF_DISABLE               (0)
#define AK09915_CNTL1_NSF_LOW                   (1 << 5)
#define AK09915_CNTL1_NSF_MID                   (2 << 5)
#define AK09915_CNTL1_NSF_HIGH                  (3 << 5)
#define AK09915_CNTL1_WATERMARK_1               (0)
#define AK09915_CNTL1_WATERMARK_2               (1)
#define AK09915_CNTL1_WATERMARK_10              (9)
#define AK09915_CNTL1_WATERMARK_20              (19)
#define AK09915_CNTL1_WATERMARK_30              (29)

#define AK09915_CNTL2_POWER_DOWN                (0x0)   /* 0b00000 */
#define AK09915_CNTL2_POWER_SINGLE              (0x01)  /* 0b00001 */
#define AK09915_CNTL2_POWER_CONTINUOUS_10HZ     (0x02)  /* 0b00010 */
#define AK09915_CNTL2_POWER_CONTINUOUS_20HZ     (0x04)  /* 0b00100 */
#define AK09915_CNTL2_POWER_CONTINUOUS_50HZ     (0x06)  /* 0b00110 */
#define AK09915_CNTL2_POWER_CONTINUOUS_100HZ    (0x08)  /* 0b01000 */
#define AK09915_CNTL2_POWER_CONTINUOUS_200HZ    (0x0A)  /* 0b01010 */
#define AK09915_CNTL2_POWER_CONTINUOUS_1HZ      (0x0C)  /* 0b01100 */
#define AK09915_CNTL2_POWER_SELFTEST            (0x10)  /* 0b10000 */


#define AK09915_CNTL2_BIT_SDR                   (1 << 6)
#define AK09915_CNTL2_BIT_FIFO                  (1 << 7)

#define AK09915_CNTL3_BIT_SRST                  (1 << 0)

/* Data come from AKM data sheet */
#define AK09915_SELFTEST_MAX_X                  (200)
#define AK09915_SELFTEST_MIN_X                  (-200)
#define AK09915_SELFTEST_MAX_Y                  (200)
#define AK09915_SELFTEST_MIN_Y                  (-200)
#define AK09915_SELFTEST_MAX_Z                  (-200)
#define AK09915_SELFTEST_MIN_Z                  (-800)

#define AK09915_I2C_CLOCK                       (400)
#define AK09915_RESOLUTION                      (16)
#define AK09915_MIN_ODR                         (1.5625f)
#define AK09915_MAX_ODR                         (200.0f)
#define AK09915_MAX_RANGE                       (4912)
#define AK09915_MAX_CURRENT                     (0.9f)

#define AK09915_ODR_THRESHOLD_FOR_SDR           (110.0f)

#endif /*~ AK09915MAGSENSOR_H */

/** @}*/
