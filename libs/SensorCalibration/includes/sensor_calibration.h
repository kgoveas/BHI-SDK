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
 * @file        sensor_calibration.h
 *
 * @date        12/20/2018
 *
 * @brief       Header file of sensor_calibration.c
 *
 */

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

/*********************************************************************/
/* system header files */
#include <types.h>

/*********************************************************************/
/* local macro definitions */
#define GYRO_ODR_FACTOR_SCALING (0.03f / 2048.0f) /* Scaling factor is +/- 3% over a range of a signed 12bit value */
#define CONVERT_GYRO_ODR_FACTOR_FROM_OTP(x) (1.0f / (1.0f + ((x) * GYRO_ODR_FACTOR_SCALING))) /* Convert 12bit OTP value into an actual factor for the Gyro ODR */

typedef struct sensor_calibration_data {
    float gyro_sense_factors[3];
    float gyro_odr_factor;
    SInt16 accel_offset_2g[3];
    SInt16 accel_offset_4g[3];
    SInt16 accel_offset_8g[3];
    SInt16 accel_offset_16g[3];
    UInt8 otp_crc;
    bool otp_crc_ok;
} sensor_calibration_data_t;

extern sensor_calibration_data_t SPECIAL_DATA sensor_calibration_data;

/*!
 * @brief Set sensor calibration data by default values
 *
 * @param[out] sensor_calibration_data_local : a pointer to sensor_calibration_data_t
 *
 * @return None
 */
void sensor_calibration_data_set_default(sensor_calibration_data_t* sensor_calibration_data_local);

/*!
 * @brief Get gyro sensor sense factors
 *
 * @param[out] x    sense factor for x axis
 * @param[out] y    sense factor for y axis
 * @param[out] z    sense factor for z axis
 *
 * @return None
 */
void get_gyro_sense_factors(float *x, float *y, float *z);

/*!
 * @brief Set gyro sensor sense factors
 *
 * @param[in] x    sense factor for x axis
 * @param[in] y    sense factor for y axis
 * @param[in] z    sense factor for z axis
 *
 * @return None
 */
void set_gyro_sense_factors(float x, float y, float z);

#endif /* !SENSOR_CALIBRATION_H */
