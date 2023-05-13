////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/SensorInterface/includes/SensorAPIInternal.h
///
/// @project    EM7189
///
/// @brief      Internal functions and data types used by the sensor interface.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2014-2018 EM Microelectronic
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

#ifndef __SENSOR_API_INTERNAL_H_
#define __SENSOR_API_INTERNAL_H_

#include <SensorAPI.h>

// Priority M gets mapped to PRIORITY_LOWEST+1
#define NUM_PRIORITIES  (((PRIORITY_LOWEST+1) - PRIORITY_2) + 1)
#define PRI_IDX(i) (SATURATE(PRIORITY_LOWEST+1, i, PRIORITY_2) - PRIORITY_2)

typedef struct
{
    VirtualSensorDescriptor* first;
    VirtualSensorDescriptor* last;
} VirtualPriorityList_t;

extern VirtualPriorityList_t gVirtTrigByPriority[NUM_PRIORITIES];

extern UInt8 gVirtualSensorIndex[SENSOR_TYPE_MAX_DESCRIPTORS];
extern bool     gNeedsReset;

#define SensorStatusPending     SensorStatusFrameworkReserved

#define NUM_RESETS_BEFORE_ERROR 2   /* Error out on the 3rd reset */
#define TIMEOUT_MULTIPLIER      3

void JLI shutdownPhysicalSensor(PhysicalSensorDescriptor* sensor);

void JLI shutdownTimerSensor(const TimerSensorDescriptor* sensor);
void JLI enableTimerSensor(const TimerSensorDescriptor* sensor);

void JLI determineVirtualSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

void JLI sensorTimerHandler(SensorStatus stat, void* data); // used for timer based sensors.

void JLI triggerSensor(VirtualSensorDescriptor* sensor, void* data, bool task);

void JLI triggerSensorsInternal(SensorDescriptorHeader* source, bool needCal, bool task);

void clearTrigger(SensorDescriptorHeader* sensor);

bool JLI handleTriggerList(SensorDescriptorHeader* source, UInt8 priority);

bool JLI parseSensorDrivers(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

UInt8 JLI checkSensorRates(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

void JLI performSelfTest(UInt8 sensorID);

void JLI performSensorFOC(UInt8 sensorID);

bool JLI updatePowerMode(PhysicalSensorDescriptor* phys);

bool JLI updateDynamicRange(PhysicalSensorDescriptor* phys);

void JLI updatePhysicalSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys);

void JLI updateTimerSensors(VirtualSensorDescriptor* virt, UInt32 numVirt);

void JLI reportFrameworkEvent(UInt8 sensor_id, UInt8 event);

void JLI notifyVirtualModeChange(const bool enables[], UInt32 numSensors, VirtualSensorDescriptor* sensors);

UInt16 JLI getDefaultDynamicRange(const PhysicalSensorDescriptor* phys);

#define NUM_RESETS_BEFORE_ERROR 2   /* Error out on the 3rd reset */
#define TIMEOUT_MULTIPLIER  3
SensorStatus checkForSensorHang(VirtualSensorDescriptor* self, void* data);

void JLI reportVisableMetaEvent(const VirtualSensorDescriptor* sensor, UInt8 event, UInt8 data, SystemTime_t timestamp, bool flush);

// ONly called once, not in JLI
void reportSensorMetaEventForPhysicalSource(const PhysicalSensorDescriptor* phys, UInt8 event, UInt8 data, bool alwaysThrow);

void JLI decimateChildren(SensorDescriptorHeader* sensor, bool sendEvents);

void JLI resetHangVariables(PhysicalSensorDescriptor* phys);

void JLI triggerVirtualTask(UInt8 priority, bool task);

bool JLI addToVirtualTriggerList(VirtualSensorDescriptor *sensor, UInt8 priority);

/**
 * @fn void hook_PhysicalRate(PhysicalSensorDescriptor* phys, float* rate);
 *
 * @brief Function called to notify listeners that a rate for the physical sensor driver has been determines.
 *
 * This function is called any time a rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different physical sensor rate, it can be overwritten here.
 *
 * @param           phys    The physical sensor with a new rate.
 * @param[in,out]   rate    The Current requested rate from the host interface.
 */
void JLI hook_PhysicalRate(PhysicalSensorDescriptor* phys, float* rate);


/**
 * @fn void hook_TimerRate(VirtualSensorDescriptor* timer, float* rate);
 *
 * @brief Function called to notify listeners that a rate for the timer sensor driver has been determines.
 *
 * This function is called any time a rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different physical sensor rate, it can be overwritten here.
 *
 * @param           timer   The timer sensor with a new rate.
 * @param[in,out]   rate    The Current requested rate from the host interface / framework.
 */
void JLI hook_TimerRate(VirtualSensorDescriptor* timer, float* rate);


/**
 * @fn void hook_PhysicalRateChanged(PhysicalSensorDescriptor* phys, float rate);
 *
 * @brief Function called to notify listeners that physical sensor driver rate has changed.
 *
 * This function is called any time a rate for a physical sensor driver has changed.
 *
 * @param   phys        The physical sensor with a new rate.
 * @param   rate        The actual rate of the sensor driver.
 */
void JLI hook_PhysicalRateChanged(PhysicalSensorDescriptor* phys, float rate);

/**
 * @fn void hook_PhysicalRangeChanged(PhysicalSensorDescriptor* phys, UInt16 range);
 *
 * @brief Function called to notify listeners that physical sensor driver range has changed.
 *
 * This function is called any time the dynamic range for a physical sensor driver has changed.
 *
 * @param   phys        The physical sensor with a new rate.
 * @param   range        The actual range of the sensor driver.
 */
void JLI hook_PhysicalRangeChanged(PhysicalSensorDescriptor* phys, UInt16 range);

/**
 * @fn void hook_VirtualSensorsDetermined();
 *
 * @brief Function called to notify listeners that the host requested virtual drivers have changed.
 */
void JLI hook_VirtualSensorsDetermined(void);

/**
 * @fn void hook_OverrideMaxRate(SensorDescriptorHeader* sensor, float* rate);
 *
 * @brief Function called to notify listeners that a max rate for the timer sensor driver has been determines.
 *
 * This function is called any time a max rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different maximal physical sensor rate, it can be overwritten here.
 *
 * @param           sensor   The sensor with a new maximal rate.
 * @param[in,out]   rate    The Current requested rate from the host interface / framework.
 */
void JLI hook_OverrideMaxRate(SensorDescriptorHeader* sensor, float* rate);

/**
 * @fn void hook_updatePhysicalState(PhysicalSensorDescriptor* phys);
 *
 * @brief Function called to notify listeners that a physical state has changed.
 *
 * This function is called any time a state of physical sensor driver is changed. In the event that
 *      the hook implementer needs a different state of physical sensor rate, it can be done here.
 *
 * @param   phys        The physical sensor.
 */
void JLI hook_updatePhysicalState(PhysicalSensorDescriptor* phys);

/**
 * @fn void hook_PhysicalRate(PhysicalSensorDescriptor* phys, float* rate);
 *
 * @brief Function called to notify listeners that a power state of the physical sensor driver will be updated.
 *
 * This function is called any time a power state of a physical sensor driver will be updated.
 *      The hook implementer determines a required power state here.
 *
 * @param   phys                The physical sensor with a new rate.
 * @retval  SensorPowerMode     The determined power mode for the setting by sensor interface.
 */
SensorPowerMode JLI hook_determinePowerState(PhysicalSensorDescriptor* phys);

#endif /* __SENSOR_API_INTERNAL_H_ */
