////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterface/includes/host.h
///
/// @project    EM7189
///
/// @brief      Macros for host communication
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
/// @cond
///
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// 1. Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// 2. Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
///
////////////////////////////////////////////////////////////////////////////////
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @endcond
////////////////////////////////////////////////////////////////////////////////
#ifndef _HOST_H
#define _HOST_H

#include <types.h>
#include <interrupts.h>
#include <gpio.h>
#include <SensorAPI.h>



#if defined(_ARCVER) || defined(_FRAMEWORK)

/**
 * @fn void initHostInterface(void);
 *
 * @brief Initialize the host interface.
 *
 * This function is called during firmware startup to initialize the host interface.
 * @retval      TRUE    The host interface initialized.
 * @retval      FALSE   The host interface failed to initialize.
 */
bool JLI initHostInterface(void);

/**
 * @fn void initHostInterrupt(void);
 *
 * @brief This configures the interrupt pin using the host interrupt control settings.
 */
void JLI initHostInterrupt(void);

/**
 * @fn bool announceHIReady(void);
 *
 * @brief   Signals to the host that the host interface is now ready.
 */
void JLI announceHIReady(void);

/**
 * @fn bool requestHostTest(void);
 *
 * @brief       Determines if the host has requested a host
 *              interface test be performed.
 *
 * @retval      TRUE    A host test has been requested.
 * @retval      FALSE   No host test has been requested.
 */
bool JLI requestHostTest(void);

/**
 * @fn float getRequestedRate(const SensorDescriptorHeader* sensor);
 *
 * @brief       Return the requested sample rate of the driver from the host.
 *
 * @param       sensor      The sensor driver to determine the rate for.
 *
 * @returns     The requested rate, in Hz (0 for disabled)
 */
//lint -function( fopen(1), getRequestedRate(1) ) // arg cannot be null
float JLI getRequestedRate(const SensorDescriptorHeader* sensor);

/**
 * @fn float getAndClearRequestedRate(const SensorDescriptorHeader* sensor);
 *
 * @brief       Return the requested sample rate of the driver from the host.
 *
 * Note: This causes host interface to believe that the sample rate has not changed.
 *
 * @param       sensor      The sensor driver to determine the rate for.
 *
 * @returns     The requested rate, in Hz (0 for disabled)
 */
//lint -function( fopen(1), getAndClearRequestedRate(1) ) // arg cannot be null
static float ALWAYS_INLINE getAndClearRequestedRate(const SensorDescriptorHeader* sensor)
{
    return getRequestedRate(sensor);
}

/**
 * @fn void updateRequestedRate(SensorDescriptorHeader* sensor, float rate);
 *
 * @brief       Modifies the requested sample rate of the driver from the host.
 *
 * @param       sensor      The sensor driver to determine the rate for.
 * @param       rate        The updated requested rate.
 */
//lint -function( fopen(1), updateRequestedRate(1) ) // arg cannot be null
void JLI updateRequestedRate(SensorDescriptorHeader* sensor, float rate);

/**
 * @fn void disableSensorUpdates(void);
 *
 * @brief       Disables update from the host interface for sensor rate changes.
 *  Note: After this function is called, multiple calls to getRequestedRate
 * <b>must</b> return the same value.
 */
void JLI disableSensorUpdates(void);

/**
 * @fn void enableSensorUpdates(void);
 *
 * @brief       Allow sensor updates from the host. This is the default state.
 *      NOTE: Enable must be called the <b>same</b> amount of times as disable
 *                     before updates are actually enabled.
 */
void JLI enableSensorUpdates(void);

/**
 * @brief       Used to determine if command processing is disabled.
 *
 * @retval      TRUE    Command processing is paused.
 * @retval      FALSE   Command processing is allowed.
 */
bool JLI isCommandProcessingPaused(void);

/**
 * @brief       Used to determine if sensor updates are currently disabled.
 *
 * @retval      TRUE    Sensor updates are disabled.
 * @retval      FALSE   Sensor updates are allowed.
 */
static bool ALWAYS_INLINE areSensorUpdatesDisabled(void)
{
    return isCommandProcessingPaused();
}

/**
 * @fn bool reportEvent(const VirtualSensorDescriptor *p, const void* data);
 *
 * @brief       Reports the specified event to the host.  The
 *              event id must have been registered previously as
 *              a valid event type.  This is done automatically
 *              for customer drivers when they are initialized,
 *              and includes the number of bytes of data for the
 *              event.  So, callers to reportEvent() must pass a
 *              data buffer.
 *
 *              Note: If a sensor is disabled, the event will be ignored.
 *
 * @param       p           Virtual sensor descriptor
 * @param[in]   data        The event contents to send to the host.
 *
 * @retval      TRUE        The event was sent to the host.
 * @retval      FALSE       The event was ignored.
 */
bool JLI reportEvent(const VirtualSensorDescriptor *p, const void* data);

/**
 * @fn bool reportEventAlways(const VirtualSensorDescriptor *p, const void* data);
 *
 * @brief       Reports the specified event to the host.  The
 *              event id must have been registered previously as
 *              a valid event type.  This is done automatically
 *              for customer drivers when they are initialized,
 *              and includes the number of bytes of data for the
 *              event.  So, callers to reportEventAlways() must pass a
 *              data buffer
 *
 * @param       p           Virtual sensor descriptor
 * @param[in]   data        The event contents to send to the host.
 *
 * @retval      TRUE        The event was sent to the host.
 * @retval      FALSE       The event was ignored.
 */
bool JLI reportEventAlways(const VirtualSensorDescriptor *p, const void* data);

/**
 * @brief       Reports a passed in meta event using the host interface.
 * @param       event       The meta event id to report.
 * @param       data        The meta event data to report.
 * @param       timestamp   The timestamp associated with the event.
 * @param       report_now  If the meta event should be reported immediately.
 * @retval      TRUE        The event was reported successfully.
 * @retval      FALSE       The event was failed to be reported.
 */
bool JLI reportMetaEvent(UInt8 event, const void *data, SystemTime_t timestamp, bool report_now);

typedef struct __attribute__((packed)) MetaEventInitialized_t {
        UInt8 event;    /**< The Event Id: META_EVENT_INITIALIZED */
        UInt16 version; /**< The firmware version. */
        UInt8 padding;  /**< Unused, padding. */
} MetaEventInitialized_t;


/**
 * @fn UInt16 getRequestedDynamicRange(SensorDescriptorHeader* sensor);
 *
 * @brief       Return the requested dynamic range of the driver from the host.
 *
 * @param       sensor      The sensor driver to determine the rage for.
 *
 * @returns     The requested range.
 */
//lint -function( fopen(1), getRequestedDynamicRange(1) ) // arg cannot be null
UInt16 JLI getRequestedDynamicRange(SensorDescriptorHeader* sensor);

/**
 * @fn void reportSensorStatus(SensorDescriptorHeader* sensor, SensorStatus status);
 *
 * @brief       Reports the status of the sensor to the host.
 *
 * @param       sensor      The sensor driver to report.
 * @param       status      The status of the sensor.
 */
//lint -function( fopen(1), reportSensorStatus(1) ) // arg cannot be null
void JLI reportSensorStatus(SensorDescriptorHeader* sensor, SensorStatus status);

/**
 * @brief   When called, this causes sensor driver self test results to be reported to the host.
 *
 * @param sensor    The sensor driver with results.
 * @param status    The result of the self test.
 * @param x_offset  Sensor specific offset information.
 * @param y_offset  Sensor specific offset information.
 * @param z_offset  Sensor specific offset information.
 */
void JLI reportSelfTestResults(const SensorDescriptorHeader *sensor, SensorStatus status, SInt16 x_offset, SInt16 y_offset, SInt16 z_offset);

/**
 * @brief   When called, this causes sensor driver FOC results to be reported to the host.
 *
 * @param sensor    The sensor driver with results.
 * @param status    The result of the FOC.
 * @param x         Sensor specific axis information.
 * @param y         Sensor specific axis information.
 * @param z         Sensor specific axis information.
 */
void JLI reportFOCResults(const SensorDescriptorHeader *sensor, SensorStatus status, SInt16 x, SInt16 y, SInt16 z);

/**
 * \brief
 * \param sensor_type -
 * \param x -
 * \param y -
 * \param z -
 * \param bsx_time_stamp -
 * \param time_stamp -
 * \param sequence -
 * \return void JLI -
 */
void JLI logBSXDoSteps(UInt8 sensor_type, float x, float y, float z, SInt64 bsx_time_stamp, UInt8 sequence, SystemTime_t time_stamp);

/**
 * \brief
 * \param sensor_id -
 * \param sample_rate -
 * \param result -
 * \param n_phys_cfgs -
 * \param sid0 -
 * \param rate0 -
 * \param sid1 -
 * \param rate1 -
 * \param sid2 -
 * \param rate2 -
 * \return void JLI -
 */
void JLI logBSXUpdateSub(UInt8 sensor_id, float sample_rate, UInt8 result, UInt8 n_phys_cfgs, UInt8 sid0, float rate0, UInt8 sid1, float rate1, UInt8 sid2, float rate2);

/**
 * @fn UInt8 frameworkToHostPowerModes(SensorPowerMode mode)
 *
 * @brief       Translated between the SensorAPI power modes and the host power modes.
 *
 * @param       mode        The mode to translate.
 */
UInt8 JLI frameworkToHostPowerModes(SensorPowerMode mode);


/**
 * @fn bool registerHostPin(UInt8 gpio);
 *
 * @brief   Notify the host interface that the specified gpio pin should be used to signal an interrupt..
 *
 * @param   gpio    The GPIO pin to use as an interrupt request.
 *
 * @retval  TRUE    The host interface has acknowledged the new GPIO pin.
 * @retval  FALSE   The host interface is unable to use the requested GPIO pin.
 */
bool JLI registerHostPin(UInt8 gpio);


/**
 * @fn UInt8 handleHostInterface(UInt32 reason);
 *
 * @brief   Function called every time the firmware returns to
 *              the PRIORITY_M state, after waking from sleep.
 * @param   reason      Why this is being called.
 * @retval  SUCCESS     The host interface was handled correctly.
 * @retval  Others      An error occurred in the host interface.
 */
UInt8 JLI handleHostInterface(UInt32 reason);

/**
 * @fn void registerHIFCommCallbackForTest(void);
 *
 * @brief   Register the function called to process host commands
 *            for driver unit test.
 */
void JLI registerHIFCommCallbackForTest(void);


/** Parameter Pages defined for the host interface. **/
#define PARAM_PAGE_NONE          (0)     /**< @brief empty page, no parameters */
#define PARAM_PAGE_SYSTEM        (1)     /**< @brief system-related parameters */
#define PARAM_PAGE_ALGORITHM     (2)     /**< @brief algorithm-related parameters */
#define PARAM_PAGE_SENSOR_INFO   (3)     /**< @brief sensor-related info parameters */
#define PARAM_PAGE_SENSOR_CONFIG (5)     /**< @brief sensor-related config parameters */

/**
 * @fn paramWriteHandler_t registerWriteParamHandler(UInt8 page, paramWriteHandler_t handler);
 *
 * @brief   This function is used to register a new Param IO load page handler.
 *              The handler should load data from the buffer {length bytes}, and use it to update internal configuration.
 *              The handler should return TRUE if the parameter id was handled, and FALSE if it doesn't exist.
 *
 * @param   page    The Param IO page to handle.
 * @param   handler The function to be called whenever a parameter is being loaded from the specified page.
 *
 * @return  The old handler for the page or NULL if the page does not exist or did not previously have a handler
 */
typedef bool (*paramWriteHandler_t)(UInt8 param, UInt16 length, UInt8 buffer[]); /**< @brief The load parameter page handler */
paramWriteHandler_t JLI registerWriteParamHandler(UInt8 page, paramWriteHandler_t handler);

/**
 * @fn paramReadHandler_t registerReadParamHandler(UInt8 page, paramReadHandler_t handler);
 *
 * @brief   This function is used to register a new Param IO load page handler.
 *              The handler should save data to the buffer.
 *              The handler should return TRUE if the parameter id was handled, and FALSE if it doesn't exist.
 *              If FALSE was returned no buffer data will be outputted to the host.
 *
 * @param   page    The Param IO page to handle.
 * @param   handler The function to be called whenever a parameter is being readd into the specified page.
 *
 * @return  The old handler for the page or NULL if the page does not exist or did not previously have a handler
 */
typedef bool (*paramReadHandler_t)(UInt8 param, UInt16 length, UInt8 buffer[], UInt16 *ret_length); /**< @brief The read parameter page handler */
paramReadHandler_t JLI registerReadParamHandler(UInt8 page, paramReadHandler_t handler);

extern paramWriteHandler_t JLI lookupWriteParamHandler(UInt8 page);
extern paramReadHandler_t JLI lookupReadParamHandler(UInt8 page);

/**
 * @fn bool writeParameter(UInt8 phase, UInt8 page, UInt8 parameter, UInt16 length, UInt8 buffer[]);
 *
 * @brief   This function is used to programatically perform a param io write.
 *
 * NOTE: This function should not be used to update a sensors requested rate or range.
 *
 * @param   phase       What comms state this was provided from
 * @param   page        The Param IO page to write data into.
 * @param   parameter   The page specfic parameter to write data into.
 * @param   length      The number of bytes in the buffer to write.
 * @param   buffer      The parameter data to write.
 *
 * @retval  TRUE        The parameter was written properly
 * @retval  FALSE       The handler was not able to be written
 */
bool JLI writeParameter(UInt8 phase, UInt8 page, UInt8 parameter, UInt16 length, UInt8 buffer[]);

/**
 * @fn bool readParameter(UInt8 phase, UInt8 page, UInt8 parameter, UInt16 length, UInt8 buffer[], UInt16 *ret_length);
 *
 * @brief   This function is used to programatically perform a param io read.
 *
 * @param   phase       What comms state this was provided from
 * @param   page        The Param IO page to read data from.
 * @param   parameter   The page specfic parameter to read data from.
 * @param   length      The number of bytes available in the buffer to read.
 * @param   buffer      The buffer to read data into. This should be a 16 byte, pre-zeroed array.
 * @param   ret_length  The number of bytes placed into the buffer.
 *
 * @retval  TRUE        The parameter was read properly
 * @retval  FALSE       The handler was not able to be read
 */
bool JLI readParameter(UInt8 phase, UInt8 page, UInt8 parameter, UInt16 length, UInt8 buffer[], UInt16 *ret_length);

/** @cond */
bool JLI registerHostInterfaceMemory(void);
/** @endcond */


/**
 * @fn UInt32 nextHostHandleTime(void);
 *
 * @brief   This function returns the maximum number of system ticks that can elapse
 *          before the host interface needs to be handled.
 *
 * @retval  0       The host interface does not need to be handled.
 * @retval  others  The host interface should be called no later than the current system time plus the return value
 */
SystemTime_t JLI nextHostHandleTime(void);


/**
 * @fn bool isAPSuspended(void);
 *
 * @brief Determine if the host is suspended.
 *
 * @retval  TRUE    suspended
 * @retval  FALSE   not suspended
 */
bool JLI isAPSuspended(void);

#endif /* _ARCVER */

#endif /* _HOST_H */
