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
 * @file            TMG4903LightSensor.h
 *
 * @date          12/20/2018
 *
 * @brief          Header file of TMG4903 light
 *
 */

/*!
 *@addtogroup TMG4903 physical driver
 *@brief
 *@{*/

#ifndef __TMG4903_SENSOR_H_
#define __TMG4903_SENSOR_H_

/*********************************************************************/
/* macro definitions */
#define TMG4903_REG_PART2_NUM                       128

#define TMG4903_INIT_RETRY                          3
#define TMG4903_REG_REV_ID                          0x91
#define TMG4903_REG_CHIP_ID                         0x92
#define TMG4903_CHIP_ID                             0xB8
#define TMG4903_REV_ID                              0x02
#define TMG4903_REVDEV_ID                           0xB802

#define TMG4903_REG_ENABLE                          0x80
#define TMG4903_REG_PERSIS                          0x8C /* bit7~bit4: Proximity interrupt peristence, bit3~bit0 : ALS interrupt peristence */

#define TMG4903_REG_CFG0                            0x8D /* bit5:Low Power Idle, bit2: Wait Long Enable */
#define TMG4903_REG_PGCFG0                          0x8E /* bit7~bit6:Proximity Pulse Length, bit5~bit0: Proximity Pulse Count. */
#define TMG4903_REG_PGCFG1                          0x8F /* bit7~bit6:Proximity Gain Control, bit4~bit1: Proximity LED Drive Strength. */
#define TMG4903_REG_CFG1                            0x90 /* Bit7:Proximity Saturation Interrupt Enable, bit6:ALS Saturation Interrupt Enable, bit1~bit0: ALS and Color Gain Control. */

#define TMG4903_REG_STATUS                          0x93
#define TMG4903_REG_STATUS2                         0x9E

#define TMG4903_REG_RLOW                            0x96
#define TMG4903_REG_RHIGH                           0x97
#define TMG4903_REG_GLOW                            0x98
#define TMG4903_REG_GHIGH                           0x99
#define TMG4903_REG_BLOW                            0x9A
#define TMG4903_REG_BHIGH                           0x9B

#define TMG4903_STATUS2_PVALID                      7    /* Proximity Valid. */
#define TMG4903_STATUS2_AVALID                      6    /* ALS Valid. */
#define TMG4903_STATUS2_ASAT_DIGITAL                4    /* ALS Digital Saturation.*/
#define TMG4903_STATUS2_ASAT_ANALOG                 3    /* ALS Analog Saturation. */
#define TMG4903_STATUS2_PGSAT_ADC                   2    /* Proximity/Gesture ADC Saturation. */
#define TMG4903_STATUS2_PGSAT_REFLECTIVE            1    /* Proximity/Gesture Reflective Saturation.*/
#define TMG4903_STATUS2_PGSAT_AMBIENT               0    /* Proximity/Gesture Ambient Saturation. */

#define TMG4903_REG_CFG2                            0x9F
#define TMG4903_CFG2_PMASK_E                        7    /* Proximity Mask East */
#define TMG4903_CFG2_PMASK_W                        6    /* Proximity Mask West */
#define TMG4903_CFG2_PMASK_S                        5    /* Proximity Mask South*/
#define TMG4903_CFG2_PMASK_N                        4    /* Proximity Mask North */
#define TMG4903_CFG2_AMASK                          3    /* ALS Mask */
#define TMG4903_REG_CFG3                            0xAB /*Bit6:Use Proximity Photodiodes for ALS Measurement, bit4:Sleep After Interrupt.*/
#define TMG4903_REG_CFG4                            0xAC
#define TMG4903_REG_CFG5                            0xAD
#define TMG4903_CFG4_ALS_INT_DIRECT                 7    /*ALS Interrupt Direct.*/
#define TMG4903_CFG4_ALS_INT_DIRECT_GPIO            6    /*ALS Interrupt Direct on GPIO Pin*/
#define TMG4903_CFG4_PROXIMITY_INT_DIRECT           5    /*Proximity Interrupt Direct.*/
#define TMG4903_CFG4_PROXIMITY_INT_DIRECT_GPIO      4    /*Proximity Interrupt Direct on GPIO Pin*/
#define TMG4903_CFG5_LONG_LTFSTOP_DISCARD_ALS       5    /*Long Disruption Discard ALS.*/
#define TMG4903_CFG5_DISABLE_IR_CORRECTION          3
#define TMG4903_CFG5_PROXIMITY_FILTER_DOWNSAMPLE    2    /*Proximity Filter Downsample.*/
#define TMG4903_CFG5_PROXIMITY_FILTER_SIZE          1    /*Proximity Filter Size.*/
#define TMG4903_CFG5_PROXIMITY_FILTER               0    /*Proximity Filter.*/

#define TMG4903_REG_STATUS3                         0xB3 /*bit1: Sleep-After-Interrupt Active.*/
#define TMG4903_REG_CONTROL                         0xBC /*bit0:Sleep-After-Interrupt Active Clear*/
#define TMG4903_REG_AZ_CONFIG                       0xD6 /*ALS Autozero Frequency.*/
#define TMG4903_REG_CALIB                           0xD7 /*Start Offset Calibration*/
#define TMG4903_REG_CALIBCFG0                       0xD8
#define TMG4903_CALIBCFG0_DCAVG_AUTO_OFFSET_ADJUST  6    /*DC Averaging Auto Offset Adjust.*/
#define TMG4903_CALIBCFG0_ELECTRICAL_CALIBRATION    4    /*Enable Electrical Calibration.*/
#define TMG4903_CALIBCFG0_BINSRCH_SKIP              3    /*Binary Search Skip.*/
#define TMG4903_CALIBCFG0_DCAVG_ITERATIONS               /*bit2~bit0 : DC Averaging Iterations.*/

#define TMG4903_REG_CALIBCFG1                       0xD9
#define TMG4903_REG_CALIBCFG2                       0xDA
#define TMG4903_REG_CALIBCFG3                       0xDB
#define TMG4903_REG_CALIBSTAT                       0xDC

#define TMG4903_REG_INTENAB                         0xDD
#define TMG4903_REG_INTCLEAR                        0xDE
#define TMG4903_ASAT                                7
#define TMG4903_PGSAT                               6
#define TMG4903_PROXIMITY                           5
#define TMG4903_ALS                                 4
#define TMG4903_IRB                                 3
#define TMG4903_GEST                                2
#define TMG4903_CALIB                               1

#define TMG4903_REG_PG_OFFSET                       0xC0
#define TMG4903_REG_PG_BASELINE                     0xD0
#define TMG4903_REG_CRGB_DATA                       0x94
#define TMG4903_REG_PROXIMITY_DATA                  0x9C

#define TMG4903_CFG0_LOWPOWER                       5
#define TMG4903_CFG0_WLONG                          2

#define TMG4903_ENABLE_IRBEAM                       7
#define TMG4903_ENABLE_GESTURE                      6
#define TMG4903_ENABLE_PROXIMITY_INTERRUPT          5
#define TMG4903_ENABLE_ALS_INTERRUPT                4
#define TMG4903_ENABLE_WAIT                         3
#define TMG4903_ENABLE_PROXIMITY                    2
#define TMG4903_ENABLE_ALS                          1
#define TMG4903_ENABLE_POWERON                      0

#define TMG4903_REG_ALS_ITIME                       0x81
#define TMG4903_REG_PROXIMITY_ITIME                 0x82
#define TMG4903_REG_WTIME                           0x83
#define TMG4903_REG_ALS_L_THLD_LOWBYTE              0x84
#define TMG4903_REG_ALS_L_THLD_HIGHBYTE             0x85
#define TMG4903_REG_ALS_H_THLD_LOWBYTE              0x86
#define TMG4903_REG_ALS_H_THLD_HIGHBYTE             0x87
#define TMG4903_REG_PROXIMITY_L_THLD_LOWBYTE        0x88
#define TMG4903_REG_PROXIMITY_L_THLD_HIGHBYTE       0x89
#define TMG4903_REG_PROXIMITY_H_THLD_LOWBYTE        0x8A
#define TMG4903_REG_PROXIMITY_H_THLD_HIGHBYTE       0x8B

#define ALS_H_THLD                                  800
#define ALS_L_THLD                                  500
#define ALS_DATA_NUM                                4

#define TMG4903_REG_PROXIMITY_SAMPLE_TIME           0x82
#define TMG4903_PROXIMITY_AWAY                      0x00
#define TMG4903_PROXIMITY_CLOSE                     0x01
#define TMG4903_PROXIMITY_INIT                      0xFF

#define TMG4903_PROXIMITY_THRES_LOW                 0x0090
#define TMG4903_PROXIMITY_THRES_HIGH                0x0120

#define TMG4903_TIMER_INTERVAL_MS                   300

#define TMG4903_C_COEF                              863
#define TMG4903_R_COEF                              (-554)
#define TMG4903_G_COEF                              1722
#define TMG4903_B_COEF                              (-2108)

#define TMG4903_ALS_ITIME                           0xDC /*0xF1*/
#define TMG4903_ATIME_PER_100                       278
#define TMG4903_AGAIN_MASK                          0x03
#define TMG4903_D_FACTOR                            16

#define TMG4903_RATE_MAX                            1
#define TMG4903_RATE_MIN                            1
#define TMG4903_DYNAMIC_RANGE_MAX                   255 /* dummy */
#define TMG4903_SCALE_FACTOR                        (1.0F)
#define TMG4903_CURRENT                             (0.130F)

#define TMG4903_PHY_SENSOR_FAMILY   {SENSOR_TYPE_INPUT_LIGHT, SENSOR_TYPE_INPUT_PROXIMITY}

/*!< max one in the view of uint32_t */
#ifndef MAX_UNSIGNED
#define MAX_UNSIGNED(x, y)                          ((uint32_t)(x) > (uint32_t)(y) ? (x) : (y))
#endif

/* do not change the order of elements, they are restricted by HW device and byte alignment */
typedef struct tmg4903_reg_data_type
{
    /* CRGB Data Registers (0x94 - 0x9B), are stored as 16-bit values */
    uint16_t crgb_data[ALS_DATA_NUM];
    /* Proximity Data Registers (0x9C - 0x9D), Proximity data is stored as a 14-bit value (two bytes). */
    uint16_t  proximity_raw;
    uint8_t   reg_cfg1;
    uint8_t   reg_cfg2;
    uint32_t  cpl;
    /* this is a bool flag, put here to utility spare space */
    bool tmg4903_chip_inited;
    /* Status Register (STATUS 0x93) */
    uint8_t status;
} __attribute__((packed))tmg4903_reg_data_t;

/*********************************************************************/
/* global variables declaration */
extern tmg4903_reg_data_t tmg4903_reg_data_raw;

/*********************************************************************/
/* external functions declaration */

/*!
 * @brief
 * read and write register for tmg4903 driver
 *
 * @param[in] device     : Current physical sensor
 * @param[in] reg         : Register addr
 * @param[in] int_mask : Bit position
 * @param[in] enable     : Flag to set or clean
 *
 * @return None
 */
void tmg4903_update_reg(const Device* device, uint8_t reg, uint8_t int_mask, bool enable);

/*!
 * @brief Initiate tmg4903 physical driver
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 *    SensorOK                                 The sensor has already been initialized
 *    SensorErrorUnexpectedDevice    The sensor did not return expected results.
 *    SensorErrorNonExistant             No device
 */
SensorStatus tmg4903_initialize_sensor(PhysicalSensorDescriptor* self);

/*!
 * @brief Set power mode for tmg4903 physical driver
 *
 * @param[in] power_mode : Power mode to be set
 * @param[in] self              : Current physical sensor descriptor
 *
 * @return Power mode
 */
SensorPowerMode tmg4903_device_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor* self);

/*!
 * @brief tmg4903 interrupt operation
 *
 * @param[in] flag   : Enable flag of interrupt
 * @param[in] self   : Current physical sensor descriptor
 *
 * @return Result of execution
 */
SensorStatus tmg4903_origin_interrupts_op(bool flag, PhysicalSensorDescriptor* self);

#endif

/** @}*/
