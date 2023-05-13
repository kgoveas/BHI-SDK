////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/sensor_descriptors.h
///
/// @project    EM7189
///
/// @brief      data structures for sensor samples
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

#ifndef _SENSOR_DESCRIPTORS_H_
#define _SENSOR_DESCRIPTORS_H_

/**
 * \brief get length of an event
 * \param event_type - the type to look up
 * \param p - virtual descriptor
 * \return UInt16 - the size in bytes or 0 if invalid
 */
extern UInt16 getEventSize(UInt8 event_type, const VirtualSensorDescriptor *p);

/**
 * \brief configure the latency of a sensor
 * \param p - virtual descriptor
 * \param latency - the value to change to
 */
extern void setSensorLatency(VirtualSensorDescriptor *p, UInt32 latency);

/**
 * \brief retrieve the configured latency of a sensor
 * \param p - virtual descriptor
 * \return latency - the value to change to
 */
extern UInt32 getSensorLatency(const VirtualSensorDescriptor *p);

/**
 * \brief find out of this event should wake the AP
 * \param p - virtual descriptor
 * \return UInt8 - TRUE if should wake it up
 */
extern UInt8 JLI shouldEventWakeupAP(const VirtualSensorDescriptor *p);

/**
 * \brief find out of this event should wake the AP
 * \param event_type - the event in question
 * \return UInt8 - TRUE if should wake it up
 */
extern UInt8 JLI shouldEventWakeupAPByEvent(UInt8 event_type);

/**
 * \brief find out of this event is a normal sensor or a control event (e.g. timestamp or meta event)
 * \param event_type - the event in question
 * \return UInt8 - TRUE if should wake it up
 */
extern UInt8 JLI isNormalSensor(UInt8 event_type);

#endif
