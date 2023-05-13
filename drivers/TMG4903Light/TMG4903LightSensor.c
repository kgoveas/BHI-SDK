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
 * @file            TMG4903LightSensor.c
 *
 * @date          12/20/2018
 *
 * @brief          Physical sensor of TMG4903 light
 *
 */

/*!
 *@addtogroup TMG4903 physical driver
 *@brief
 *@{*/

/*********************************************************************/
/* system header files */
#include <I2CDriverInterface.h>
#include <Timer.h>
#include <string.h>
#include <arc.h>
#include <host.h>

/*********************************************************************/
/* local header files */
#include "TMG4903LightSensor.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV      (1u)

/*********************************************************************/
/* static variables */
static uint8_t tmg4903_reg_value[TMG4903_REG_PART2_NUM] = {0};
/* 1 byte status, 4 * 2 bytes CRGB Data, 2 bytes Proximity Data */
static uint8_t tmg4903_read_buf[10]; /* use for reading 4903 data registers */
static uint16_t light_data = 0;
static SensorPowerMode tmg4903_phy_power_mode = SensorPowerModePowerDown;
static SensorPowerMode tmg4903_light_power_mode = SensorPowerModePowerDown;
static float tmg4903_light_sample_rate = 0;
sensorSampleDataCallback tmg4903_light_callback;

tmg4903_reg_data_t tmg4903_reg_data_raw =
{
    .tmg4903_chip_inited = FALSE,
};

static uint8_t als_gains[] =
{
    1,
    4,
    16,
    64
};

uint8_t tmg4903_reg_original_value[TMG4903_REG_PART2_NUM] =
{
    0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x4f, 0x80,
    0x00, 0x02, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x00, 0x00,
    0x00, 0x8f, 0x80, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x44, 0x0c, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*********************************************************************/
/* function declaration */
static void tmg4903_config_sensor(const PhysicalSensorDescriptor *self);

/*********************************************************************/
/* function */

/* Allow deprecated API calls from the initialize function. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/*!
 * @brief Initiate tmg4903 phisical driver
 */
SensorStatus tmg4903_initialize_sensor(PhysicalSensorDescriptor *self)
{
    uint8_t buffer[2];

    /* read chip id and version id in one shot */
    if (read_data(&self->device, TMG4903_REG_REV_ID, buffer, 2) != SensorOK)
    {
        return SensorErrorNonExistant;
    }

    /* check chip id */
    while (buffer[1] != TMG4903_CHIP_ID)
    {
        static uint8_t tmg4903_init_retry = 0;

        if (tmg4903_init_retry >= TMG4903_INIT_RETRY)
        {
            return SensorErrorUnexpectedDevice;
        }

        (void)delayums(50);
        tmg4903_init_retry++;
        (void)read_data(&self->device, TMG4903_REG_REV_ID, buffer, 2);
    }

    if (tmg4903_reg_data_raw.tmg4903_chip_inited == FALSE)
    {
        tmg4903_config_sensor(self);
        tmg4903_reg_data_raw.tmg4903_chip_inited = TRUE;
    }

    return SensorOK;
}

SensorStatus tmg4903_light_initialize_sensor(PhysicalSensorDescriptor *self)
{
    return tmg4903_initialize_sensor(self);
}

/*!
 * @brief Config parameter for tmg4903
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return None
 */
static void tmg4903_config_sensor(const PhysicalSensorDescriptor *self)
{
    uint8_t buffer[(TMG4903_REG_PERSIS - TMG4903_REG_ENABLE) + 1];
    uint32_t cpl;

    /* set the device */
    *buffer = 0;    /* disable  TMG4903_STATUS_AINT & TMG4903_STATUS_PINT */
    /* ATIME Integration Time Register (ATIME 0x81), bigger and shorter ALS */
    buffer[TMG4903_REG_ALS_ITIME - TMG4903_REG_ENABLE]   = TMG4903_ALS_ITIME;

    /* Proximity Sample Time Register (PTIME 0x82) , smaller and shorter PROXIMITY */
    buffer[TMG4903_REG_PROXIMITY_ITIME - TMG4903_REG_ENABLE] = 0x23/*0x10*/;

    /* Wait Time Register (WTIME 0x83)                bigger and shorter ALS & PROXIMITY */
    buffer[TMG4903_REG_WTIME - TMG4903_REG_ENABLE]       = TMG4903_ALS_ITIME;

    buffer[TMG4903_REG_ALS_L_THLD_LOWBYTE - TMG4903_REG_ENABLE]  = 0xfe;
    buffer[TMG4903_REG_ALS_L_THLD_HIGHBYTE - TMG4903_REG_ENABLE] = 0xff;

    buffer[TMG4903_REG_ALS_H_THLD_LOWBYTE - TMG4903_REG_ENABLE]  = 0xff;
    buffer[TMG4903_REG_ALS_H_THLD_HIGHBYTE - TMG4903_REG_ENABLE] = 0xff;

    buffer[TMG4903_REG_PROXIMITY_L_THLD_LOWBYTE - TMG4903_REG_ENABLE]  = TMG4903_PROXIMITY_THRES_LOW & 0xFF;
    buffer[TMG4903_REG_PROXIMITY_L_THLD_HIGHBYTE - TMG4903_REG_ENABLE] = (uint8_t)(((uint16_t)TMG4903_PROXIMITY_THRES_LOW) >> 8) & 0xFF;  //lint !e572

    buffer[TMG4903_REG_PROXIMITY_H_THLD_LOWBYTE - TMG4903_REG_ENABLE]  = TMG4903_PROXIMITY_THRES_HIGH & 0xFF;
    buffer[TMG4903_REG_PROXIMITY_H_THLD_HIGHBYTE - TMG4903_REG_ENABLE] = (uint8_t)(((uint16_t)TMG4903_PROXIMITY_THRES_HIGH) >> 8) & 0xFF;

    /* Interrupt Persistence Register (PERS 0x8C) */
    buffer[TMG4903_REG_PERSIS - TMG4903_REG_ENABLE] = 0x54/*0x88*/;
    (void)write_data(&self->device, TMG4903_REG_ENABLE, buffer, (TMG4903_REG_PERSIS - TMG4903_REG_ENABLE) + 1);
    memcpy(&tmg4903_reg_value[TMG4903_REG_ENABLE - TMG4903_REG_PART2_NUM], buffer,
            (TMG4903_REG_PERSIS - TMG4903_REG_ENABLE) + 1);

    (void)read_data(&self->device, TMG4903_REG_CFG1, (uint8_t*)&tmg4903_reg_data_raw.reg_cfg1, sizeof(uint8_t));
    (void)read_data(&self->device, TMG4903_REG_CFG2, (uint8_t*)&tmg4903_reg_data_raw.reg_cfg2, sizeof(uint8_t));

    cpl = 256 - TMG4903_ALS_ITIME;
    cpl *= TMG4903_ATIME_PER_100;
    cpl /= 100;
    cpl *= als_gains[(tmg4903_reg_data_raw.reg_cfg1 & TMG4903_AGAIN_MASK)];

    tmg4903_reg_data_raw.cpl = cpl;
}

#pragma GCC diagnostic pop

/*!
 * @brief Read and write register for tmg4903 driver
 */
void tmg4903_update_reg(const Device *device, uint8_t reg, uint8_t int_mask, bool enable)
{
    uint8_t buffer;

    buffer = tmg4903_reg_value[reg - TMG4903_REG_PART2_NUM];
    if (enable)
    {
        buffer |= int_mask;
    }
    else
    {
        buffer &= ~int_mask;
    }

    write_data_inline_nonblocking(device, reg, &buffer, sizeof(uint8_t), NULL, NULL);
    tmg4903_reg_value[reg - TMG4903_REG_PART2_NUM] = buffer;
}

/*!
 * @brief tmg4903 interrupt operation
 */
SensorStatus tmg4903_origin_interrupts_op(bool flag, PhysicalSensorDescriptor *self)
{
    if (self != NULL)
    {
        if (self->int_enabled != flag)
        {
            self->int_enabled = flag;
        }
        return SensorOK;
    }

    return SensorUnknownError;
}

/*!
 * @brief Enable tmg4903 light sensor interrupt
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
static SensorStatus tmg4903_light_enable_interrupts(PhysicalSensorDescriptor *self)
{
    /* turn on interrupt switch */
    tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                          1u << TMG4903_ENABLE_ALS_INTERRUPT | 1u << TMG4903_ENABLE_ALS, TRUE);
    return tmg4903_origin_interrupts_op(TRUE, self);
}

/*!
 * @brief Disable tmg4903 light sensor interrupt
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
static SensorStatus tmg4903_light_disable_interrupts(PhysicalSensorDescriptor *self)
{
    /* turn on interrupt switch */
    tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                          1u << TMG4903_ENABLE_ALS_INTERRUPT | 1u << TMG4903_ENABLE_ALS, FALSE);
    return tmg4903_origin_interrupts_op(FALSE, self);
}

/*!
 * @brief Do self-test for tmg4903 light sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
SensorStatus tmg4903_light_selftest(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    /* add self test code here */
    return SensorSelfTestPassed;
}

/*!
 * @brief Get sample rate for tmg4903 light sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Sample rate
 */
static float tmg4903_light_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return tmg4903_light_sample_rate;
}

/*!
 * @brief Get scale factor of tmg4903 light sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Scale factor
 */
static float tmg4903_light_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    /* change the factor basing on the gain */
    return TMG4903_SCALE_FACTOR;
}

/*!
 * @brief Get dytnamic range of tmg4903 light driver
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Dynamic range
 */
static uint16_t tmg4903_light_get_dynamic_range(PhysicalSensorDescriptor *self)
{
    /* This is the dynamic range according to the gain setting.*/
    return self->maxDynamicRange;
}

/*!
 * @brief Set power mode of tmg4903 physical device
 *
 * @param[in] power_mode : Power mode to be set
 * @param[in] self       : Current physical sensor descriptor
 *
 * @return Power mode
 */
SensorPowerMode tmg4903_phy_set_power_mode(SensorPowerMode power_mode, const PhysicalSensorDescriptor *self)
{
    if ((self != NULL) && (power_mode != tmg4903_phy_power_mode))
    {
        bool enable_flag;

        switch (power_mode)
        {
            case SensorPowerModePowerDown:
            case SensorPowerModeSuspend:
                enable_flag = FALSE;
                break;

            case SensorPowerModeLowPowerActive:
            case SensorPowerModeActive:
                enable_flag = TRUE;
                break;

            default:
                return tmg4903_phy_power_mode;
        }

        tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE, 1u << TMG4903_ENABLE_POWERON, enable_flag);

        tmg4903_phy_power_mode = power_mode;
    }

    return tmg4903_phy_power_mode;
}

/*!
 * @brief Set power mode for tmg4903 physical driver
 */
SensorPowerMode tmg4903_device_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    bool not_in_family_flag = TRUE;
    uint32_t num_phys = getNumPhysicalSensorDescriptors();
    PhysicalSensorDescriptor* sensors = getPhysicalSensorDescriptors();
    PhysicalSensorDescriptor* phy_p;
    SensorPowerMode highest_power_mode = SensorPowerModePowerDown;
    SensorPowerMode sensor_power_mode;
    uint32_t family_table[] = TMG4903_PHY_SENSOR_FAMILY;
    uint32_t index = sizeof(family_table) / sizeof(family_table[0]);

    /* find the highest power mode of whole tmg4903 phy sensor family */
    while (index--)
    {
        phy_p = getPhysicalSensorDescriptorByType(family_table[index], sensors, num_phys);

        if (phy_p == NULL)
        {
            /* this device is not exist */
            continue;
        }

        if (phy_p == self)
        {
            not_in_family_flag = FALSE;
        }
        else
        {
            sensor_power_mode = phy_p->get_power_mode(phy_p);
            highest_power_mode = MAX_UNSIGNED(highest_power_mode, sensor_power_mode);
        }
    }

    /* if the sensor is not in this family, set SensorPowerModePowerDown */
    if (not_in_family_flag)
    {
        highest_power_mode = SensorPowerModePowerDown;
    }

    /* always set the tmg4903 phy core in highest power mode */
    return tmg4903_phy_set_power_mode(MAX_UNSIGNED(highest_power_mode, mode), self);
}

/*!
 * @brief Set power mode of tmg4903 light driver.
 *
 * @param[in] mode : Power Mode to be set
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Power mode of the physical sensor.
 */
static SensorPowerMode tmg4903_light_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    bool update_flag = FALSE;
    bool enable_flag = FALSE;

    if (mode != tmg4903_light_power_mode)
    {
        switch (mode)
        {
            case SensorPowerModePowerDown:
            case SensorPowerModeSuspend:
                update_flag = TRUE;
                enable_flag = FALSE;
                break;

            case SensorPowerModeLowPowerActive:
            case SensorPowerModeActive:
                update_flag = TRUE;
                enable_flag = TRUE;
                break;

            case SensorPowerModeFOC:
            {
                /* offset */
                int16_t offsets[] = { 0, 0, 0 };
                SensorStatus tmg4903_status = SensorErrorFOCFailed;

                /* add foc subroutine here */
                reportFOCResults((SensorDescriptorHeader*)(self), tmg4903_status, offsets[0], offsets[1], offsets[2]);
                reportSensorStatus((SensorDescriptorHeader*)(self), tmg4903_status);
                break;
            }

            default:
                return tmg4903_light_power_mode;
        }

    }

    if (update_flag)
    {
        /* update device power mode */
        (void)tmg4903_device_set_power_mode(mode, self);

        /* turn on interrupt switch */
        if (enable_flag)
        {
            tmg4903_update_reg(&self->device, TMG4903_REG_ENABLE,
                                  1u << TMG4903_ENABLE_ALS_INTERRUPT | 1u << TMG4903_ENABLE_ALS, TRUE);
        }

        tmg4903_light_power_mode = mode;
    }
    sensorPowerModeChanged(SensorOK, self);

    return tmg4903_light_power_mode;
}

/*!
 * @brief Get power mode for tmg4903 light sensor
 *
 * @param[in] self : Current physical sensor descriptor
 *
 * @return Result of execution
 */
static SensorPowerMode tmg4903_light_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return tmg4903_light_power_mode;
}

/*!
 * @brief Set sample rate for tmg4903 light driver
 *
 * @param[in] self : current physical sensor descriptor
 *
 * @return Sample rate
 */
static float tmg4903_light_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    float maxRate = self->maxRate;

    if (sample_rate > maxRate)
    {
        sample_rate = maxRate;
    }

    tmg4903_light_sample_rate = sample_rate;
    sensorRateChanged(SensorOK, self);

    return tmg4903_light_sample_rate;
}

/*!
 * @brief Callback to handle tmg4903 light raw data
 *
 * @param[in] res  : Result of callback function
 * @param[in] data : Transferred data
 *
 * @return None
 */
static void tmg4903_handle_als_data_callback(SensorStatus res,  void *data)
{
    int32_t lux;
    uint16_t clear_raw, red_raw, blue_raw, green_raw;
    PhysicalSensorDescriptor *sensor = (PhysicalSensorDescriptor *)data;

    if (res != SensorOK)
    {
        return; /* error happens on bus, abort */
    }

    tmg4903_reg_data_raw.crgb_data[0] = (tmg4903_read_buf[1] << 8) | tmg4903_read_buf[0];
    tmg4903_reg_data_raw.crgb_data[1] = (tmg4903_read_buf[3] << 8) | tmg4903_read_buf[2];
    tmg4903_reg_data_raw.crgb_data[2] = (tmg4903_read_buf[5] << 8) | tmg4903_read_buf[4];
    tmg4903_reg_data_raw.crgb_data[3] = (tmg4903_read_buf[7] << 8) | tmg4903_read_buf[6];
    tmg4903_reg_data_raw.proximity_raw = (tmg4903_read_buf[9] << 8) | tmg4903_read_buf[8];
    clear_raw = tmg4903_reg_data_raw.crgb_data[0];
    red_raw   = tmg4903_reg_data_raw.crgb_data[1];
    green_raw = tmg4903_reg_data_raw.crgb_data[2];
    blue_raw  = tmg4903_reg_data_raw.crgb_data[3];

    lux = clear_raw * TMG4903_C_COEF + red_raw * TMG4903_R_COEF +
          green_raw * TMG4903_G_COEF + blue_raw * TMG4903_B_COEF;

    lux *= TMG4903_D_FACTOR;
    lux /= (int32_t)tmg4903_reg_data_raw.cpl;
    lux += 50;
    lux /= 100;

    if (lux < 0)
    {
        lux = 0;
    }
    light_data = (uint16_t)lux;

    if ((tmg4903_reg_data_raw.status & (1u << TMG4903_ALS)) && sensor->int_enabled)
    {
        /* trigger the virtual sensor(s) of light sensors */
        tmg4903_light_callback((SensorDescriptorHeader*)sensor);
    }

    if (tmg4903_reg_data_raw.status & (1u << TMG4903_PROXIMITY))
    {
        /* trigger the virtal sensor(s) of proximity sensor */
        PhysicalSensorDescriptor* phy_p = getPhysicalSensorDescriptorByType(SENSOR_TYPE_INPUT_PROXIMITY,
                                          getPhysicalSensorDescriptors(), getNumPhysicalSensorDescriptors());

        if ((phy_p != NULL) && (phy_p->int_enabled))
        {
            sensorInterruptHandler(getSystemTime(), phy_p);
        }
    }
}

/*!
 * @brief Callback to read tmg4903 light raw data
 *
 * @param[in] res  : Result of callback function
 * @param[in] data : Rransferred data
 *
 * @return None
 */
static void tmg4903_dis_int_callback(SensorStatus res, void *data)
{
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)data;

    if (res != SensorOK)
    {
        return; /* error happens on bus, abort */
    }

    (void)read_data_nonblocking(&self->device, TMG4903_REG_CRGB_DATA, tmg4903_read_buf,
                                   sizeof(tmg4903_read_buf), tmg4903_handle_als_data_callback, self);
}

/*!
 * @brief Handle sensor data for tmg4903
 *
 * @param[in] res    : Result of callback function
 * @param[in] sensor : Current sensor discriptor
 *
 * @return None
 */
static void tmg4903_handle_sensor_data(SensorStatus res, void *sensor)
{
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)sensor;
    if (res != SensorOK)
    {
        return; /* error happens on bus, abort */
    }

    /* clean the triggered interrupt, no consequent operation */
    (void)write_data_nonblocking(&self->device, TMG4903_REG_INTCLEAR, (uint8_t*)&tmg4903_reg_data_raw.status,
                                    sizeof(uint8_t), tmg4903_dis_int_callback, self);
}

/*!
 * @brief Get sample data of light sensor
 *
 * @param[in] callback : Callback function
 * @param[in] self     : Current physical sensor descriptor
 *
 * @return None
 */
static void tmg4903_light_get_sample_data(sensorSampleDataCallback callback, PhysicalSensorDescriptor *self)
{
    /* cache callback function, reuse element expansionData */
    tmg4903_light_callback = callback;

    /*read 1 byte status, 4 * 2 bytes CRGB Data, 2 bytes Proximity Data in one shot, enhance efficiency */
    (void)read_data_nonblocking(&self->device, TMG4903_REG_STATUS, (uint8_t*)&tmg4903_reg_data_raw.status,
                                   sizeof(uint8_t), tmg4903_handle_sensor_data, self);
}

/********************************************************************************/
/* descriptor of physical sensor */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor tmg4903_light_descriptor =
{
    .device =
    {
        .options =
        {
            .i2c =
            {
                .hasClockStretching = 0,
                .maxClock = 400, /* 400kHz */
            },//lint !e785
        },//lint !e785

        .interface =
        {
            .options = InterfaceAnyI2C,
        },

        .irqEdge = IRQ_EDGE_FALLING,
    },//lint !e785

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = 16,
    },//lint !e785

    .type =
    {
        .value = SENSOR_TYPE_INPUT_LIGHT,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
        .no_hang = TRUE,
    },//lint !e785

    .initialize = tmg4903_light_initialize_sensor,

    .set_power_mode = tmg4903_light_set_power_mode,
    .get_power_mode = tmg4903_light_get_power_mode,

    .set_sample_rate = tmg4903_light_set_sample_rate,
    .get_sample_rate = tmg4903_light_get_sample_rate,

    .enable_interrupts = tmg4903_light_enable_interrupts,
    .disable_interrupts = tmg4903_light_disable_interrupts,

    .get_scale_factor = tmg4903_light_get_scale_factor,
    .get_sample_data = tmg4903_light_get_sample_data,

    .set_dynamic_range = NULL,
    .get_dynamic_range = tmg4903_light_get_dynamic_range,

    .sensorData = (void*)&light_data,

    .maxDynamicRange = TMG4903_DYNAMIC_RANGE_MAX,
    .maxRate = TMG4903_RATE_MAX,   /* 1 Hz */
    .minRate = TMG4903_RATE_MAX,   /* 1 Hz */
    .maxCurrent = TMG4903_CURRENT, /* 0.13 mA */
};//lint !e785

/** @}*/
