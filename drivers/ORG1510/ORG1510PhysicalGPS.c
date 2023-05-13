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
 */

/*! \file ORG1510AccelSensor.c
   \brief  Sensor drivers of ORG1510 */

/*!
 * @addtogroup ORG1510_GPS_driver
 * @brief
 * @{*/

/*********************************************************************/
/* system header includes */
#include <SensorAPI.h>
#include <Timer.h>
#include <string.h>
#include <arc.h>
#include <host.h>
#include <stdio.h>

/*********************************************************************/
/* local header files */
#include "ORG1510.h"

/*********************************************************************/
/* local macro definitions */

#define DRIVER_REV                      (1u)
/*! @name Physical sensor descriptor */
#define this                            (self)
/*! @name Delay time defination */
#define GPS_DELAY_5MS                   (5)    /**< 5ms */
#define GPS_DELAY_10MS                  (10)    /**< 10ms */
#define GPS_DELAY_20MS                  (20)    /**< 20ms */
#define GPS_DELAY_50MS                  (50)    /**< 20ms */
#define GPS_DELAY_100MS                 (100)    /**< 100ms */
#define GPS_DELAY_200MS                 (200)    /**< 200ms */
#define GPS_DELAY_500MS                 (500)    /**< 500ms */
#define GPS_DELAY_1S                    (1000)    /**< 1000ms */
#define GPS_DELAY_1_5S                  (1500)    /**< 1500ms */
#define GPS_DELAY_2S                    (2000)    /**< 2000ms */
#define GPS_DELAY_3S                    (3000)    /**< 3000ms */
#define GPS_DELAY_4S                    (4000)    /**< 4000ms */
#define GPS_DELAY_5S                    (5000)    /**< 50000ms */
#define GPS_DELAY_6S                    (6000)    /**< 60000ms */
#define GPS_DELAY_7S                    (7000)    /**< 70000ms */
#define GPS_DELAY_8S                    (8000)    /**< 80000ms */
#define GPS_DELAY_9S                    (9000)    /**< 9000ms */
#define GPS_DELAY_10S                   (10000)    /**< 10000ms */
#define GPS_DELAY_12S                   (12000)    /**< 12000ms */
#define GPS_DELAY_15S                   (15000)    /**< 15000ms */
#define GPS_DELAY_20S                   (20000)    /**< 20000ms */

#define GPS_POLL_TIMER_INDEX_IDLE       (-1)
#define GPS_POLL_TIMER_INTERVAL         (GPS_DELAY_10MS)    /**< 100Hz */
#define GPS_POLL_TIMER_RATE             (GPS_DELAY_1S/GPS_POLL_TIMER_INTERVAL)
#define GPS_DELAY_ACTIVE                (GPS_DELAY_5S)
#define GPS_DELAY_SLEEP                 (GPS_DELAY_5S)
#define GPS_GOTO_SLEEP_DELAY            (GPS_DELAY_10S)

/*! @name GPS buffer size defination */
#define GPS_BUFFER_SIZE                 (256)   /*!< GPS buffer size */
#define GPS_TRANSFER_SIZE               (20)    /*!< GPS transfer size */
#define GPS_MSG_BUFFER_SIZE             (5)     /*!< GPS message buffer size */

/*! @name GPS state defination */
#define GPS_STATE_ACTIVE                (1)     /*!< GPS active state */
#define GPS_STATE_SLEEP                 (0)     /*!< GPS sleep state */

/*********************************************************************/
/*! @name Enumeration Definitions */

/**
 * @brief GPS run state
 */
typedef enum gps_run_state_enum
{
    GPSRunStateReset = 0,               /**< Reset. */
    GPSRunStateInit,                    /**< Initial. */
    GPSRunStateSleep,                   /**< Sleep. */
    GPSRunStateSleep2Run,               /**< Sleep to run. */
    GPSRunStateRun2Sleep,               /**< Run to sleep. */
    GPSRunStateRun,                     /**< Run. */
    GPSRunStateMax,                     /**< Max. */
} gps_run_state_enum_t;

/**
 * @brief GPS SPI transfer state
 */
typedef enum gps_spi_state_enum
{
    GPSSPI_IDLE = 0,                    /**< Idle. */
    GPSSPI_IN_USE,                      /**< Transferring. */
    GPSSPI_COMPLETE,                    /**< Complete. */
    GPSSPI_Max,                         /**< Max. */
} gps_spi_state_enum_t;

/**
 * @brief GPS SPI transfer state
 */
typedef enum gps_parse_state_enum
{
    GPSParseNone = 0,                   /**< None. */
    GPSParseNEMA,                       /**< NEMA. */
    GPSParseOSP,                        /**< OSP. */
    GPSParseMax,                        /**< Max. */
} gps_parse_state_enum_t;

/**
 * @brief GPS NMEA message parse state
 */
typedef enum nema_parse_state_enum
{
    ParseNEMANone = 0,                  /**< None. */
    ParseNEMAStart,                     /**< Start. */
    ParseNEMAID,                        /**< Id. */
    ParseNEMAPayload,                   /**< Payload. */
    ParseNEMAChecksum,                  /**< Checksum. */
    ParseNEMACRLF,                      /**< CRLF. */
    ParseNEMAError,                     /**< Error. */
    ParseNEMAMax,                       /**< Max. */
} nema_parse_state_enum_t;

/**
 * @brief GPS OSP message parse state
 */
typedef enum osp_parse_state_enum
{
    ParseOSPNone = 0,                   /**< None. */
    ParseOSPMax,                        /**< Max. */
} osp_parse_state_enum_t;

/**
 * @brief GPS NMEA message id
 */
typedef enum nema_msg_id_enum
{
    NMEA_MSG_NONE = 0,                  /**< NMEA message id: none. */
    NMEA_MSG_GGA,                       /**< NMEA message id: GGA. */
    NMEA_MSG_GLL,                       /**< NMEA message id: GLL. */
    NMEA_MSG_GSA,                       /**< NMEA message id: GSA. */
    NMEA_MSG_GSV,                       /**< NMEA message id: GSV. */
    NMEA_MSG_GNS,                       /**< NMEA message id: GNS. */
    NMEA_MSG_RMC,                       /**< NMEA message id: RMC. */
    NMEA_MSG_150,                       /**< NMEA message id: 150. */
    NMEA_MSG_160,                       /**< NMEA message id: 160. */
    NMEA_MSG_Max,                       /**< NMEA message id: max. */
} nema_msg_id_enum_t;

/**
 * @brief GPS NMEA talker id
 */
typedef enum nema_talker_id_enum
{
    NMEA_TALKER_None = 0,               /**< NMEA talker id: reset. */
    NMEA_TALKER_GP,                     /**< NMEA talker id: GP. */
    NMEA_TALKER_GL,                     /**< NMEA talker id: GL. */
    NMEA_TALKER_GN,                     /**< NMEA talker id: GN. */
    NMEA_TALKER_BD,                     /**< NMEA talker id: BD. */
    NMEA_TALKER_PSRF,                   /**< NMEA talker id: PSRF. */
    NMEA_TALKER_Max,                    /**< NMEA talker id: max. */
} nema_talker_id_enum_t;

/*********************************************************************/
/*! @name Structure Definitions */
/*!
 * @brief NMEA message structure
 */
typedef struct gps_nmea_msg_struct
{
    uint8_t talker_id;
    uint8_t msg_id;
    uint8_t header[10];
    uint8_t payload[256];
    uint8_t header_length;
    uint16_t payload_length;
    uint8_t checksum[3];
    uint8_t checksum_length;
    uint8_t checksum_cal;
    uint8_t validation;
    uint8_t buffer[512];
    uint16_t buffer_length;
} gps_nmea_msg_struct_t;

/*!
 * @brief NMEA message id structure
 */
typedef struct gps_nmea_msg_id_struct
{
    uint8_t msg[4];
    uint8_t msg_id;
} gps_nmea_msg_id_struct_t;

/*!
 * @brief GPS NMEA message exchange buffer
 */
typedef struct GPS_NMEA_MSG
{
    uint16_t msg_length;
    uint16_t msg_transfered;
    uint8_t buffer[512];
} gps_nmea_msg_t;

/*!
 * @brief GPS packet report to virtgps
 */
typedef struct GPS_PACKET
{
    uint16_t msg_length;
    uint16_t msg_transfered;
    uint8_t packet_size;
    uint8_t protocol_type;
    uint8_t packet[GPS_TRANSFER_SIZE];
} __attribute__ ((packed)) gps_packet_t;

/*********************************************************************/
/*! @name Static Variable Definitions */

static SInt8 g_gps_poll_timer = -1;
static float g_gps_sample_rate = 0;
/*! physical sensor power mode */
static SensorPowerMode g_gps_PowerMode;
/*! gps run state */
static gps_run_state_enum_t g_gps_run_state = GPSRunStateReset;
/*! gps message parse state*/
static gps_parse_state_enum_t g_gps_parse_state = GPSParseNone;
/*! gps nmea message parse state */
static nema_parse_state_enum_t g_nmea_parse_state = ParseNEMAStart;
/*! gps nmea message object */
static gps_nmea_msg_struct_t g_nmea_msg = {0};
/*! gps run delay (active + sleep) */
static UInt32 g_gps_delay = 0;
/*! gps active delay */
static UInt32 g_gps_delay_active = 0;
/*! gps callback complete indication */
static uint8_t g_gps_callback_complete = GPSSPI_IDLE;
/*! gps sending data to virtgps indication */
static uint8_t g_gps_sending_data = 0;
/*! gps active state (active or sleep) */
static uint8_t g_gps_active = GPS_STATE_SLEEP;
/*! gps read buffer */
static uint8_t g_gps_rd_buf[GPS_BUFFER_SIZE] = {0};
/*! gps write buffer */
static uint8_t g_gps_wr_buf[GPS_BUFFER_SIZE] =
{
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
    0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4, 0xA7, 0xB4,
};/* 16x16 */
/*! gps packet obj report to virtgps */
static gps_packet_t g_gps_packet = {0};

/*********************************************************************/
/*! @name global variables definitions */

/*! gps nmea message buffer */
gps_nmea_msg_t g_gps_nema_msg_buf[GPS_MSG_BUFFER_SIZE] = {{0}};

/*!
 * @brief NMEA message id table
 */
gps_nmea_msg_id_struct_t gps_nmea_msg_id_table[] =
{
    {"GGA", NMEA_MSG_GGA},
    {"GLL", NMEA_MSG_GLL},
    {"GSA", NMEA_MSG_GSA},
    {"GSV", NMEA_MSG_GSV},
    {"GNS", NMEA_MSG_GNS},
    {"RMC", NMEA_MSG_RMC},
    {"150", NMEA_MSG_150},
    {"160", NMEA_MSG_160},
    //{"GGA",NMEA_MSG_GGA},
};

/*********************************************************************/
/*! @name Global Function Declarations */

/*!
* @brief Convert HEX string to int format data.
*
* @param[in] s[] : pointer to HEX string
*
* @return int data.
*/
int htoi_my(char s[])
{
    int n = 0;
    int i = 0;

    while ((s[i] != '\0') && (s[i] != '\n'))
    {
        if (s[i] == '0')
        {
            if ((s[i + 1] == 'x') || (s[i + 1] == 'X')) //lint !e679
            {
                i += 2;
            }
        }
        if ((s[i] >= '0') && (s[i] <= '9'))
        {
            n = n * 16 + (s[i] - '0');
        }
        else if ((s[i] >= 'a') && (s[i] <= 'f'))
        {
            n = n * 16 + (s[i] - 'a') + 10;
        }
        else if ((s[i] >= 'A') && (s[i] <= 'F'))
        {
            n = n * 16 + (s[i] - 'A') + 10;
        }
        else
        {
            return -1;
        }
        ++i;

    }

    return n;
}

/*!
* @brief Compare string s1 and s2.
*
* @param[in] *s1 : Pointer to string s1.
* @param[in] *s2 : Pointer to string s2.
* @param[in]   n : Numbers of charactors to compare.
*
* @return result.
*/
int strncmp_my ( char *s1, char *s2, int n)
{
    if ( !n )
    {
        return (0);
    }

    while ((--n) && (*s1) && (*s1 == *s2))
    {
        s1++;
        s2++;
    }

    return ( *s1 - *s2 );
}

/*!
* @brief Set physical gps run state.
*
* @param[in] new_state : new state to set
*
* @return none.
*/
void org1510_phy_gps_set_run_state(gps_run_state_enum_t new_state)
{
    switch (new_state)
    {
        case GPSRunStateReset:
            DEBUG_PRINT("GPSReset\r\n");
            g_gps_run_state = new_state;
            break;
        case GPSRunStateInit:
            DEBUG_PRINT("GPSInit\r\n");
            g_gps_run_state = new_state;
            break;
        case GPSRunStateSleep:
            /* set reset pin to high */
            DEBUG_PRINT("GPSSleep\r\n");
            g_gps_run_state = new_state;
            break;
        case GPSRunStateSleep2Run:
            g_gps_run_state = new_state;
            DEBUG_PRINT("GPSSleep2Run\r\n");
            break;
        case GPSRunStateRun2Sleep:
            g_gps_run_state = new_state;
            DEBUG_PRINT("GPSRun2Sleep\r\n");
            break;
        case GPSRunStateRun:
            DEBUG_PRINT("GPSRun\r\n");
            g_gps_run_state = new_state;
            break;
        case GPSRunStateMax:
        default:
            DEBUG_PRINT("GPSStateMax\r\n");
            break;
    }

}

/*!
* @brief Get physical gps run state.
*
* @return current gps state.
*/
gps_run_state_enum_t org1510_phy_gps_get_run_state(void)
{
    return g_gps_run_state;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        enable interrupt of PHY device registered in EM framwork.
*
* @param[in] self_p : a pointer to sensor descriptor
*
* @return result of operation.
*/
SensorStatus org1510_phy_gps_enable_interrupts(PhysicalSensorDescriptor *self)
{
    SensorStatus result = SensorOK;

    if (this->int_enabled == FALSE)
    {
        timer_enable_interrupt(g_gps_poll_timer);
        this->int_enabled = TRUE;
    }
    return result;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        disable interrupt of PHY device registered in EM framwork.
*
* @param[in] self_p : A pointer to sensor descriptor
*
* @return result of operation.
*/
SensorStatus org1510_phy_gps_disable_interrupts(PhysicalSensorDescriptor *self)
{
    SensorStatus result = SensorUnknownError;

    if (self != NULL)
    {
        if (this->int_enabled == TRUE)
        {
            /* we don't disable timer here until gps is in sleep mode */
        }
        result = SensorOK;
    }

    return result;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        set power mode of PHY device.
*
* @param[in] mode   : the desired Power Mode meant to be set
* @param[in] self_p : a pointer to sensor descriptor
*
* @return power mode of PHY device.
*/
SensorPowerMode org1510_phy_gps_set_power_mode(SensorPowerMode mode, PhysicalSensorDescriptor *self)
{
    if (self != NULL)
    {
        switch (mode)
        {
            case SensorPowerModeSelfTest:
                /*  no self-test functionality */
                reportSensorStatus(cast_PhysicalToHeader(this), SensorSelfTestPassed);
                break;
            case SensorPowerModeSuspend:
            case SensorPowerModePowerDown:
                /*  power down physical sensor */
                g_gps_PowerMode = SensorPowerModePowerDown;
                DEBUG_PRINT("GPSPowerDown\r\n");
                break;
            case SensorPowerModeLowPowerActive:
            case SensorPowerModeActive:
                DEBUG_PRINT("GPSActive\r\n");
                org1510_phy_gps_set_run_state(GPSRunStateSleep2Run);
                /*  power on physical sensor */
                g_gps_PowerMode = SensorPowerModeActive;
                break;

            default:
                return g_gps_PowerMode;
        }
    }
    sensorPowerModeChanged(SensorOK, self);

    return g_gps_PowerMode;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        get power mode of PHY device.
*
* @param[in] self_p : a pointer to sensor descriptor
*
* @return power mode of PHY device.
*/
SensorPowerMode org1510_phy_gps_get_power_mode(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return g_gps_PowerMode;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        set sample rate of PHY device.
*
* @param[in] sample_rate :  the desired sample rate meant to be set
* @param[in] self_p      :  a pointer to sensor descriptor
*
* @return sample rate of PHY device.
*/
float org1510_phy_gps_set_sample_rate(float sample_rate, PhysicalSensorDescriptor *self)
{
    if (sample_rate > self->maxRate)
    {
        sample_rate = self->maxRate;
    }
    if ((sample_rate > 0.0F) && (sample_rate < self->minRate))
    {
        sample_rate = self->minRate;
    }
    g_gps_sample_rate = sample_rate;

    sensorRateChanged(SensorOK, this);

    return g_gps_sample_rate;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        get sample rate of PHY device.
*
* @param[in] self_p : a pointer to sensor descriptor
*
* @return sample rate of of phy device
*/
float org1510_phy_gps_get_sample_rate(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return g_gps_sample_rate;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        get scale factor of PHY device. dummy API, not defined
*
* @param[in] self_p :  a pointer to sensor descriptor
*
* @return scale factor of of phy device
*/
float org1510_phy_gps_get_scale_factor(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return ORG1510_SCALE_FACTOR;
}

/*!
* @brief An interface of EM framework, check its definition in EM manul for detailed info,
*        it sets the ORG1510 raw data read record unit and invoke ORG1510_get_sample_data_next()
*        to start raw data fetching
*
* @param[in] CallBack : call back function invoked after data have been fetched successfully
* @param[in] self_p   : a pointer to sensor descriptor
*
* @return none
*/
void org1510_phy_gps_get_sample_data(sensorSampleDataCallback CallBack, PhysicalSensorDescriptor *self)
{
    uint8_t i = 0;
    g_gps_sending_data = 1;
    for ( i = 0 ; i < GPS_MSG_BUFFER_SIZE; i++)
    {
        if (g_gps_nema_msg_buf[i].msg_length)
        {
            /* get gps message total length */
            g_gps_packet.msg_length = g_gps_nema_msg_buf[i].msg_length;
            /* get gps message transfered length */
            g_gps_packet.msg_transfered = g_gps_nema_msg_buf[i].msg_transfered;
            /* get gps message current packet length */
            if ((g_gps_nema_msg_buf[i].msg_length - g_gps_nema_msg_buf[i].msg_transfered) >= GPS_TRANSFER_SIZE)
            {
                g_gps_packet.packet_size = GPS_TRANSFER_SIZE;
            }
            else
            {
                g_gps_packet.packet_size = (g_gps_nema_msg_buf[i].msg_length - g_gps_nema_msg_buf[i].msg_transfered);
            }
            g_gps_packet.protocol_type = 0;
            /* get gps message current packet data */
            memset(g_gps_packet.packet, 0, sizeof(g_gps_packet.packet));
            memcpy(g_gps_packet.packet, &g_gps_nema_msg_buf[i].buffer[g_gps_nema_msg_buf[i].msg_transfered],
                   g_gps_packet.packet_size);
            /* increase transfered packet */
            g_gps_nema_msg_buf[i].msg_transfered += g_gps_packet.packet_size;

            /* clean transfered msg buffer */
            if (g_gps_nema_msg_buf[i].msg_transfered == g_gps_nema_msg_buf[i].msg_length)
            {
                g_gps_nema_msg_buf[i].msg_length = 0;
                g_gps_nema_msg_buf[i].msg_transfered = 0;
                memset(g_gps_nema_msg_buf[i].buffer, 0, sizeof(g_gps_nema_msg_buf[i].buffer));
            }

            CallBack((void *)self);
            break;
        }
    }

    g_gps_sending_data = 0;
    return;
}

/*!
* @brief An interface of EM framework, get parameter dynamic range of ORG1510 module.
*        check its definition in EM manul for detailed info, curretnly it is dummy API, not defined
*
* @param[in] self_p : a pointer to sensor descriptor
*
* @return result of operation
*/
uint16_t org1510_phy_gps_get_dynamic_range(PhysicalSensorDescriptor *self)
{
    UNUSED(self);
    return 0;
}

/*!
* @brief Parse and get talker id of a NMEA message.
*
* @param[in] p_nmea_msg : a pointer to a NMEA message
*
* @return result of operation
*
*  value    |  Description
* ----------|--------------
*   TRUE    | succeed to get a talker id
*   FALSE   | fail to get a talker id
*/
uint8_t org1510_phy_gps_nmea0183_parse_talker_id(gps_nmea_msg_struct_t *p_nmea_msg)
{
    uint8_t parse_state = TRUE;

    if (p_nmea_msg->header[0] == 'G') /* GP, GL, GN */
    {
        if (p_nmea_msg->header_length == 5)
        {
            switch (p_nmea_msg->header[1])
            {
                case 'P':
                    p_nmea_msg->talker_id = NMEA_TALKER_GP;
                    break;
                case 'L':
                    p_nmea_msg->talker_id = NMEA_TALKER_GL;
                    break;
                case 'N':
                    p_nmea_msg->talker_id = NMEA_TALKER_GN;
                    break;
                default:
                    parse_state = FALSE;
                    break;
            }
        }
        else
        {
            parse_state = FALSE;
        }
    }
    else if (p_nmea_msg->header[0] == 'P') /* PSRF */
    {
        if (p_nmea_msg->header_length == 7)
        {
            if (p_nmea_msg->header[1] == 'S' && p_nmea_msg->header[2] == 'R' && p_nmea_msg->header[3] == 'F')
            {
                p_nmea_msg->talker_id = NMEA_TALKER_PSRF;
            }
            else
            {
                parse_state = FALSE;
            }
        }
        else
        {
            parse_state = FALSE;
        }
    }
    else if (p_nmea_msg->header[0] == 'B') /* BD */
    {
        if (p_nmea_msg->header_length == 5)
        {
            if (p_nmea_msg->header[1] == 'D')
            {
                p_nmea_msg->talker_id = NMEA_TALKER_BD;
            }
            else
            {
                parse_state = FALSE;
            }
        }
        else
        {
            parse_state = FALSE;
        }
    }
    else
    {
        parse_state = FALSE;
    }
    return parse_state;
}

/*!
* @brief Parse and get message id of a NMEA message.
*
* @param[in] p_nmea_msg : a pointer to a NMEA message
*
* @return result of operation
*
*  value    |  Description
* ----------|--------------
*   TRUE    | succeed to get a message id
*   FALSE   | fail to get a message id
*/
uint8_t org1510_phy_gps_nmea0183_parse_msg_id(gps_nmea_msg_struct_t *p_nmea_msg)
{
    uint8_t i = 0;
    uint8_t msg_count = sizeof(gps_nmea_msg_id_table) / sizeof(gps_nmea_msg_id_struct_t);
    uint8_t parse_state = TRUE;

    if ((p_nmea_msg->talker_id == NMEA_TALKER_GP)
            || (p_nmea_msg->talker_id == NMEA_TALKER_GL)
            || (p_nmea_msg->talker_id == NMEA_TALKER_GN)
            || (p_nmea_msg->talker_id == NMEA_TALKER_BD))
    {
        for ( i = 0; i < msg_count; i++)
        {
            if (strncmp_my((char *)&p_nmea_msg->header[2], (char *)gps_nmea_msg_id_table[i].msg, 3) == 0)
            {
                p_nmea_msg->msg_id = gps_nmea_msg_id_table[i].msg_id;
                break;
            }
        }
        if ((i == msg_count) && (p_nmea_msg->msg_id == NMEA_MSG_NONE))
        {
            parse_state = FALSE;
        }
    }
    else if (p_nmea_msg->talker_id == NMEA_TALKER_PSRF)
    {
        for (i = 0; i < msg_count; i++)
        {
            if (strncmp_my((char *)&p_nmea_msg->header[4], (char *)gps_nmea_msg_id_table[i].msg, 3) == 0)
            {
                p_nmea_msg->msg_id = gps_nmea_msg_id_table[i].msg_id;
                break;
            }
        }
        if ((i == msg_count) && (p_nmea_msg->msg_id == NMEA_MSG_NONE))
        {
            parse_state = FALSE;
        }
    }
    else
    {
        parse_state = FALSE;
    }

    return parse_state;
}

/*!
* @brief Copy message to buffer.
*
* @return result of operation
*/
uint8_t org1510_phy_gps_nmea0183_copy_to_buf(void)
{
    uint8_t i = 0;
    if (g_nmea_msg.validation)
    {
        for ( i = 0; i < GPS_MSG_BUFFER_SIZE; i++)
        {
            if (g_gps_nema_msg_buf[i].msg_length == 0)
            {
                g_gps_nema_msg_buf[i].msg_length = g_nmea_msg.buffer_length;
                memcpy(g_gps_nema_msg_buf[i].buffer, g_nmea_msg.buffer, (g_nmea_msg.buffer_length));
                g_gps_nema_msg_buf[i].msg_transfered = 0;
                g_nmea_msg.validation = 0;
                return 1;
            }
        }
        if ((i == GPS_MSG_BUFFER_SIZE) && (g_nmea_msg.validation))
        {
        }
    }
    return 0;
}

/*!
* @brief Parse a NMEA message.
*
* @param[in] p_nmea_msg : a pointer to a NMEA message
*
* @return result of operation
*
*  value    |  Description
* ----------|--------------
*   1       | got a valid nmea message
*   0       | haven't got a valid nmea message
*/
uint8_t org1510_phy_gps_nmea0183_parse(uint8_t *p_data)
{
    uint8_t valid_msg = 0;
    uint8_t tmp_checksum = 0;
    static uint8_t tmp_cr_lf_state = 0;
    switch (g_nmea_parse_state)
    {
        case ParseNEMANone:
            break;
        case ParseNEMAStart:
            break;
        case ParseNEMAID:
            g_nmea_msg.buffer[g_nmea_msg.buffer_length] = *p_data;
            g_nmea_msg.buffer_length++;
            if (*p_data != ',')
            {
                g_nmea_msg.header[g_nmea_msg.header_length] = *p_data;
                g_nmea_msg.header_length++;

            }
            else
            {
                if ((org1510_phy_gps_nmea0183_parse_talker_id(&g_nmea_msg) == TRUE)
                        && (org1510_phy_gps_nmea0183_parse_msg_id(&g_nmea_msg) == TRUE))
                {
                    g_nmea_parse_state = ParseNEMAPayload;
                }
                else
                {
                    g_nmea_parse_state = ParseNEMAError;
                }
            }
            g_nmea_msg.checksum_cal ^= *p_data;
            break;
        case ParseNEMAPayload:
            g_nmea_msg.buffer[g_nmea_msg.buffer_length] = *p_data;
            g_nmea_msg.buffer_length++;
            if (*p_data != '*')
            {
                g_nmea_msg.payload[g_nmea_msg.payload_length] = *p_data;
                g_nmea_msg.payload_length++;
                g_nmea_msg.checksum_cal ^= *p_data;
            }
            else
            {
                g_nmea_parse_state = ParseNEMAChecksum;
            }
            break;
        case ParseNEMAChecksum:
            g_nmea_msg.buffer[g_nmea_msg.buffer_length] = *p_data;
            g_nmea_msg.buffer_length++;
            g_nmea_msg.checksum[g_nmea_msg.checksum_length] = *p_data;
            g_nmea_msg.checksum_length++;
            if (g_nmea_msg.checksum_length == 2)
            {
                tmp_checksum = htoi_my((char *)g_nmea_msg.checksum);
                if (g_nmea_msg.checksum_cal == tmp_checksum)
                {
                    g_nmea_parse_state = ParseNEMACRLF;
                }
                else
                {
                    g_nmea_parse_state = ParseNEMAError;
                }
            }
            break;
        case ParseNEMACRLF:
            g_nmea_msg.buffer[g_nmea_msg.buffer_length] = *p_data;
            g_nmea_msg.buffer_length++;
            if (tmp_cr_lf_state == 0)
            {
                if (*p_data == '\r')
                {
                    tmp_cr_lf_state = 1;
                }
                else
                {
                    g_nmea_parse_state = ParseNEMAError;
                }
            }
            else
            {
                if (*p_data == '\n')
                {
                    tmp_cr_lf_state = 0;
                    valid_msg = 1;
                    g_nmea_msg.validation = 1; /* a whole nema message is parsed success */
                    g_nmea_parse_state = ParseNEMAStart;
                    g_gps_parse_state = GPSParseNone;
                }
                else
                {
                    g_nmea_parse_state = ParseNEMAError;
                }
            }
            break;
        case ParseNEMAError:
        case ParseNEMAMax:
            g_nmea_parse_state = ParseNEMAStart;
            g_gps_parse_state = GPSParseNone;
            break;
    }

    if (g_nmea_parse_state == ParseNEMAError)
    {
        g_nmea_parse_state = ParseNEMAStart;
        g_gps_parse_state = GPSParseNone;
    }

    return valid_msg;
}

/*!
* @brief Parse and do some reaction for NMEA mssage PSRF.
*
* @return always 0.
*/
uint8_t org1510_phy_gps_nmea0183_psrf(void)
{

    switch (g_nmea_msg.msg_id)
    {
        case NMEA_MSG_150:
            if (g_nmea_msg.payload_length)
            {
                switch (g_nmea_msg.payload[0])
                {
                    case '0':/* not ok to send */
                        /* set gps module state to sleep */
                        g_gps_active = GPS_STATE_SLEEP;
                        break;
                    case '1':/* ok to send */
                        break;
                    default:
                        break;
                }
            }
            break;
        case NMEA_MSG_160:
            break;
        case NMEA_MSG_NONE:
        case NMEA_MSG_Max:
        default:
            break;
    }

    return 0;
}

/*!
* @brief Parse and do some reaction for NMEA.
*
* @return always 0.
*/
uint8_t org1510_phy_gps_nmea0183_analyse( void )
{
    uint8_t res = 0;

    if (g_nmea_msg.validation)
    {
        switch (g_nmea_msg.talker_id)
        {
            case NMEA_TALKER_GP:
                break;
            case NMEA_TALKER_GL:
                break;
            case NMEA_TALKER_GN:
                break;
            case NMEA_TALKER_BD:
                break;
            case NMEA_TALKER_PSRF:
                (void)org1510_phy_gps_nmea0183_psrf();
                break;
            case NMEA_TALKER_None:
            case NMEA_TALKER_Max:
            default:
                break;
        }
    }

    return res;
}

/*!
* @brief Trigger to report gps NMEA messages to virtgps sensor.
*
* @param[in] *param : pointer of physical sensor descriptor
*
* @return always 0.
*/
uint8_t org1510_phy_gps_msg_report(void *param)
{
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)param;
    uint8_t i = 0;

    for ( i = 0; i < GPS_MSG_BUFFER_SIZE; i++)
    {
        if ( (g_gps_nema_msg_buf[i].msg_length > 0) && (g_gps_sending_data == 0))
        {
            sensorInterruptHandler(getSystemTime(), this);
        }
    }

    return 0;
}

/*!
* @brief Parse gps messages one byte by one byte.
*        If successfully get a gps message(NMEA or OSP), return ture.
*        Currently, only implement NMEA.
*
* @param[in] *p_data : pointer to gps message buffer.
*
* @return Status of a valid NMEA message.
*
*  value    |  Description
* ----------|--------------
*   1       | got a valid nmea message
*   0       | haven't got a valid nmea message
*/
uint8_t org1510_phy_gps_msg_parse(uint8_t *p_data)
{
    uint8_t new_msg = 0;
    switch (g_gps_parse_state)
    {
        case GPSParseNone:
            if ((*p_data == '$') && (g_nmea_parse_state == ParseNEMAStart))
            {
                g_gps_parse_state = GPSParseNEMA;
                g_nmea_parse_state = ParseNEMAID;

                g_nmea_msg.talker_id = NMEA_TALKER_None;
                g_nmea_msg.msg_id = NMEA_MSG_NONE;
                g_nmea_msg.header_length = 0;
                memset(g_nmea_msg.header, 0, sizeof(g_nmea_msg.header));
                g_nmea_msg.payload_length = 0;
                memset(g_nmea_msg.payload, 0, sizeof(g_nmea_msg.payload));
                g_nmea_msg.checksum_cal = 0;
                g_nmea_msg.checksum_length = 0;
                memset(g_nmea_msg.checksum, 0, sizeof(g_nmea_msg.checksum));
                g_nmea_msg.buffer_length = 0;
                memset(g_nmea_msg.buffer, 0, sizeof(g_nmea_msg.buffer));

                g_nmea_msg.buffer[g_nmea_msg.buffer_length] = *p_data;
                g_nmea_msg.buffer_length++;
            }
            break;
        case GPSParseNEMA:
            if (g_nmea_msg.validation == 0)
            {
                /* parse gps data and get a whole nmea msg */
                if (org1510_phy_gps_nmea0183_parse( p_data))
                {
                    new_msg = 1;
                }
            }
            break;
        case GPSParseOSP:

            break;
        case GPSParseMax:

            break;
    }
    return new_msg;
}

/*!
* @brief Exchange data complete via spi callback function
*
* @param[in] stat : unused
* @param[in] data : unused
*
* @return none.
*/
void org1510_phy_gps_exchange_data_callback(SensorStatus stat, void *data)
{
    UNUSED(stat);
    UNUSED(data);
    g_gps_callback_complete = GPSSPI_COMPLETE;
}

/*!
* @brief Generate a pulse phase 3, generate a low level signal
*
* @param[in] stat : unused
* @param[in] data : pointer to a callback
*
* @return none.
*/
void org1510_on_off_set_low(SensorStatus stat, void *data)
{
    UNUSED(stat);
    void  *retData = (void *)SensorOK;
    void (*callback)(SensorStatus, void *) = data;

    /* Phase three, set the GPIO pin low. */
    setGPIOLow(gGPSOnOffPin);
    (void)timer_oneshot_delay(5.0f, callback, retData); /* wait 5ms before calling the original callback.*/

}

/*!
* @brief generate a pulse phase 2, generate a high level signal
*
* @param[in] stat : unused
* @param[in] data : pointer to a callback
*
* @return none.
*/
void org1510_on_off_set_high(SensorStatus stat, void *data)
{
    UNUSED(stat);
    /* Phase two, set the GPIO pin high.*/
    setGPIOHigh(gGPSOnOffPin);
    (void)timer_oneshot_delay(5.0f, org1510_on_off_set_low, data); /* wait 5ms before calling phase 3.*/
}

/*!
* @brief generate a pulse phase 1, generate a low level signal
*
* @param[in] callback    pointer to a callback
*
* @return none.
*/
void org1510_on_off_nonblocking(void (*callback)(SensorStatus, void *))
{
    /* Phase one - set the GPIO pin low.*/
    setGPIOLow(gGPSOnOffPin);
    (void)timer_oneshot_delay(1.0f, org1510_on_off_set_high, callback); /* wait 1ms before calling phase 2.*/
}

/*!
* @brief callback function, set gps to run mode
*
* @param[in] stat : unused
* @param[in] data : unused
*
* @return none.
*/
void  org1510_set_run_state_run (SensorStatus stat, void *data)
{
    UNUSED(stat);
    UNUSED(data);
    /* The on/off pulse has been sent. Set the mode to active now.*/
    g_gps_delay_active = 0;
    g_gps_delay = 0;
    g_gps_active = GPS_STATE_ACTIVE;
    org1510_phy_gps_set_run_state(GPSRunStateRun);
}

/*!
* @brief callback function, set gps to active mode
*
* @param[in] stat : unused
* @param[in] data : unused
*
* @return none.
*/
void  org1510_set_run_state_active (SensorStatus stat, void *data)
{
    UNUSED(stat);
    UNUSED(data);
    /* The on/off pulse has been sent. Set the mode to active now.*/
    g_gps_delay_active = 0;
    g_gps_delay = 0;
    g_gps_active = GPS_STATE_ACTIVE;
    /* org1510_phy_gps_set_run_state(GPSRunStateRun);*/
}

/*!
* @brief Callback function, set gps to sleep mode
*
* @param[in] stat : unused
* @param[in] data : unused
*
* @return none.
*/
void  org1510_set_run_state_sleep (SensorStatus stat, void *data)
{
    UNUSED(stat);
    UNUSED(data);
}

/**
 * @brief Timer processing function
 *
 * @param[in] status :  Usage unknown
 * @param[in] param  : Pointer passed from timer schedule function
 *
 * @return none.
 */
void org1510_timer_proc(SensorStatus status, void *param)
{
    PhysicalSensorDescriptor *self = (PhysicalSensorDescriptor *)param;
    gps_run_state_enum_t gps_run_state = org1510_phy_gps_get_run_state ();
    uint16_t i = 0;
    uint8_t wkup_new = FALSE;
    static uint8_t wkup_old = FALSE;

    UNUSED(status);

    switch (gps_run_state)
    {
        case GPSRunStateReset:
            (void)org1510_on_off_pin_init();
            (void)org1510_reset_pin_init();
            /* set reset pin to high */
            org1510_phy_gps_set_run_state(GPSRunStateSleep);
            break;
        case GPSRunStateInit:
            break;
        case GPSRunStateSleep:
            g_gps_delay_active = 0;
            g_gps_delay = 0;
            g_gps_active = GPS_STATE_SLEEP;
            timer_disable_interrupt(g_gps_poll_timer);
            this->int_enabled = FALSE;
            break;
        case GPSRunStateSleep2Run:
            org1510_on_off_nonblocking(org1510_set_run_state_run);
            break;
        case GPSRunStateRun2Sleep:
            break;
        case GPSRunStateRun:
            switch (g_gps_active)
            {
                case GPS_STATE_ACTIVE:
                    /* turn off gps into sleep mode */
                    if (g_gps_delay == GPS_DELAY_ACTIVE)
                    {
                        org1510_on_off_nonblocking(org1510_set_run_state_sleep);
                    }
                    /* exchange data from gps */
                    if (g_gps_callback_complete == GPSSPI_IDLE)
                    {
                        g_gps_callback_complete = GPSSPI_IN_USE;
                        (void)spi_transfer_bytes(&self->device.options.spi,
                                                 g_gps_wr_buf, g_gps_rd_buf, (uint8_t)GPS_TRANSFER_SIZE,
                                                 org1510_phy_gps_exchange_data_callback, g_gps_rd_buf,
                                                 self->device.interface.handle);
                    }
                    /* parse gps data */
                    if (g_gps_callback_complete == GPSSPI_COMPLETE)
                    {
                        for (i = 0; i < GPS_TRANSFER_SIZE; i++)
                        {
                            /* parse gps data and get a whole nmea msg */
                            if (org1510_phy_gps_msg_parse(&g_gps_rd_buf[i]))
                            {
                                (void)org1510_phy_gps_nmea0183_analyse();
                                /*copy a whole nmea msg to nmea buffer */
                                (void)org1510_phy_gps_nmea0183_copy_to_buf();
                            }
                        }
                        g_gps_callback_complete = GPSSPI_IDLE;
                    }

                    /* reprot gps msg to virtual gps sensor */
                    (void)org1510_phy_gps_msg_report(this);

                    g_gps_delay_active += GPS_POLL_TIMER_INTERVAL;
                    break;
                case GPS_STATE_SLEEP:
                    if ((g_gps_PowerMode == SensorPowerModePowerDown) || (g_gps_PowerMode == SensorPowerModeSuspend))
                    {
                        DEBUG_PRINT("gps_pwr_down!\r\n");
                        org1510_phy_gps_set_run_state(GPSRunStateSleep);
                    }
                    /* turn on gps into active mode */
                    if (g_gps_delay >= (g_gps_delay_active + GPS_DELAY_SLEEP))
                    {
                        org1510_on_off_nonblocking(org1510_set_run_state_active);
                    }
                    break;
                default:
                    break;
            }

            g_gps_delay += GPS_POLL_TIMER_INTERVAL;
            break;
        default:
            DEBUG_PRINT("gps sta error!\r\n");
            break;
    }

    wkup_new = org1510_wkup_pin_read ();
    if (wkup_new != wkup_old)
    {
        //if (wkup_new == TRUE)
        //{
        //    DEBUG_PRINT("GPSWKUP_ON\r\n");
        //}
        //else
        //{
        //    DEBUG_PRINT("GPSWKUP_OFF\r\n");
        //}
        wkup_old = wkup_new;
    }
}

/*!
* @brief Initialization function of ORG1510 module
*
* @param[in] self_p : a pointer to sensor descriptor
*
* @return result of operation
*/
SensorStatus org1510_phy_gps_initialize(PhysicalSensorDescriptor *self)
{
    /* overload cal matrix as auxiliary GPIO pin for now (need better mechanism -- maybe another field) */
    gGPSOnOffPin = (uint8_t)self->CalMatrix[0];
    gGPSResetPin = (uint8_t)self->CalMatrix[1];
    gGPSWkupPin = (uint8_t)self->CalMatrix[2];
    gGPS1PPPin = (uint8_t)self->CalMatrix[3];
    DEBUG_PRINT("org1510_init\r\n");
    DEBUG_PRINT("GPS_ONOFF:%d\r\n", gGPSOnOffPin);
    DEBUG_PRINT("GPS_RESET:%d\r\n", gGPSResetPin);
    DEBUG_PRINT("GPS_WKUP:%d\r\n", gGPSWkupPin);
    DEBUG_PRINT("GPS_1PP:%d\r\n", gGPS1PPPin);
    ClearWatchdog();

    (void)org1510_on_off_pin_init();
    (void)org1510_reset_pin_init();
    (void)org1510_wkup_pin_init();
    (void)org1510_1pp_pin_init();
    (void)org1510_reset_pin_high();
    /* Schedule a dummy timer */
    if (g_gps_poll_timer != GPS_POLL_TIMER_INDEX_IDLE)
    {
        timer_unschedule_polling(g_gps_poll_timer);
    }
    g_gps_poll_timer = timer_schedule_polling((float)GPS_POLL_TIMER_RATE, org1510_timer_proc, this);

    /*timer_enable_interrupt(g_gps_poll_timer);*/
    org1510_phy_gps_set_run_state(GPSRunStateSleep);

    return SensorOK;
}

/*!
 * @brief An element of EM framework check its definition in EM manul, a descriptor of target PHY devcie
 */
PHYSICAL_SENSOR_DESCRIPTOR PhysicalSensorDescriptor org1510_phy_gps_descriptor =
{
    .device =
    {
        .options =
        {
            .spi =
            {
                .maxClock = 5000, /**< 5MHz */
                .cpol = 0, /**< spi1 slave mode 1 */
                .cpha = 1, /**< spi1 slave mode 1 */
                .reg_shift = 0,
                .read_pol = 0, /**< @brief Specify if a read is signified by a 1 or a 0 in the cmd byte. */
                .read_bit = 0, /**< @brief Specify the bit posiion for the read/write bit. This is often bit 7 or 0. */
                /*.csPin = 11,*/
                /*.csLevel = 0,*/
            },
        },

        .interface =
        {
            .options = InterfaceAnySPI,
        },

        .irqEdge = IRQ_EDGE_RISING,
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
        .resolution = 16,
    },

    .type =
    {
        .value = SENSOR_TYPE_INPUT_GPS,
        .flags = DRIVER_TYPE_PHYSICAL_FLAG,
    },

    .initialize = org1510_phy_gps_initialize,

    .set_power_mode = org1510_phy_gps_set_power_mode,
    .get_power_mode = org1510_phy_gps_get_power_mode,

    .set_sample_rate = org1510_phy_gps_set_sample_rate,
    .get_sample_rate = org1510_phy_gps_get_sample_rate,

    .enable_interrupts = org1510_phy_gps_enable_interrupts,
    .disable_interrupts = org1510_phy_gps_disable_interrupts,

    .get_scale_factor = org1510_phy_gps_get_scale_factor,
    .get_sample_data = org1510_phy_gps_get_sample_data,

    .set_dynamic_range = NULL,
    .get_dynamic_range = org1510_phy_gps_get_dynamic_range,

    .sensorData = (void *)&g_gps_packet /* org1510_SensorData */,
    .maxDynamicRange = ORG1510_DYNAMIC_RANGE,
    .maxRate = ORG1510_RATE_MAX,
    .minRate = ORG1510_RATE_MIN,
    .maxCurrent = ORG1510_MAX_CURRENT,
};

/** @}*/
