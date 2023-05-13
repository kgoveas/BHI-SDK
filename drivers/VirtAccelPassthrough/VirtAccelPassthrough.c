////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtAccelPassthrough/VirtAccelPassthrough.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual accelerometer.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <host.h>
#include <hooks_support.h>
#include <FreeRTOS.h>
#include "VirtAccelPassthrough.h"

/*********************************************************************/
/* local macro definition */
#define DRIVER_REV      (1u)

/*********************************************************************/
/* static variables */
/* Variable to allow consumers to fore the rate for a tilt sensor. */
static uint8_t gNOBSXNeedAccelRate = 0;

/*********************************************************************/
/* functions */

/* Hook to ensure that the accel sensor is on at the requested rate or faster */
HOOK_DEF(PhysicalRate, accelPhysicalRate, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float* rate)
{
    UNUSED(stopHookExecution);

    if (phys->info.status.enabled_ack) /* framework or library requested ON */
    {
        uint8_t type = phys->type.value;
        /* Only override phys drivers that BSX owns. */
        if (type == BSX_INPUT_ID_ACCELERATION)
        {
            float minrate = gNOBSXNeedAccelRate;

            if (*rate < minrate)
            {
                *rate = minrate;    /* Override host rates. */
            }
        }
    }
} //lint !e818

HOOK_DEF(determinePowerState, nobsx_defaultHookDeterminePowerState, HOOK_PRIORITY_ROM, SensorPowerMode,
         PhysicalSensorDescriptor* phys)
{
    UNUSED(stopHookExecution);

    if(phys->info.status.enabled_ack)
    {
        SensorDescriptorHeader* child = phys->triggerList;
        while(child)
        {
            if (getRequestedRate(child))
            {
                return SensorPowerModeActive;
            }
            child = child->triggerList;
        }

        /* No child sensors need us on, strt up in low power mode. */
        return SensorPowerModeLowPowerActive;
    }
    else
    {
        return SensorPowerModePowerDown;
    }
} //lint !e818

HOOK_REF(PhysicalRate, accelPhysicalRate, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float* rate);
HOOK_REF(determinePowerState, nobsx_defaultHookDeterminePowerState, HOOK_PRIORITY_ROM, SensorPowerMode, PhysicalSensorDescriptor* phys);

/*!
 * @brief This function reports sensor data to host
 *
 * @param[in] self : Descriptor for virtual sensor
 * @param[in] data : physical raw data
 *
 * @return  SensorOK for sucessful data report. Other values for failure.
 */
SensorStatus accel_passthrough_handle_sensor_data(VirtualSensorDescriptor* self, void* data)
{
    output_t output;
    uint32_t *sendata = data;
    uint32_t x, y, z;

    portDISABLE_INTERRUPTS();

    x = sendata[0];
    y = sendata[1];
    z = sendata[2];

    portENABLE_INTERRUPTS();

    output.x = (int16_t)x;
    output.y = (int16_t)y;
    output.z = (int16_t)z;

    reportSensorEvent(self, &output, self->timestamp);

    return SensorOK;
} //lint !e818

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtaccel_calc_descriptor = {
    .triggerSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_ACCELERATION,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_ACCELERATION,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type =
    {
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_PASSTHROUGH),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .outputPacketSize = sizeof(output_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = accel_passthrough_handle_sensor_data,
    .mode_changed = NULL,
}; //lint !e785
