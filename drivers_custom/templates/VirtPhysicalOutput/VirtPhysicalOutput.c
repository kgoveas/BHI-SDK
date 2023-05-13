////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/templates/VirtPhysicalOutput/VirtPhysicalOutput.c
///
/// @project    EM7189
///
/// @brief      Template driver used to report physical sensor data to the host.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016-2018 EM Microelectronic
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

#include <SensorAPI.h>
#include <host.h>
#include "VirtPhysicalOutput.h"

#include <arc.h>

#include <FreeRTOS.h>

#define DRIVER_REV          (1u)
#define this                (self)

#define SENSOR_INPUT        BSX_INPUT_ID_ACCELERATION            /* Physical sensor to use for trigger source (source data) */
#define SENSOR_OUTPUT       BSX_OUTPUT_ID_ACCELERATION_RAW    /* Host output type */

#define SCALE_FACTOR        (1.0F / (1000.0F)) /* Convert from mg to g) */

/*****************************/
/*        Global Variables   */
/*****************************/

/*****************************/
/*        Static variables      */
/*****************************/

/*****************************/
/*        Support Functions     */
/*****************************/

/*****************************/
/*        Functions             */
/*****************************/

static SensorStatus handle_sensor_data(VirtualSensorDescriptor* self, void* data)
{
    output_t output;
    PhysicalSensorDescriptor* parent = cast_HeaderToPhysical(getSensorParent(cast_VirtualToHeader(this)));

    float scaleAdjustment = self->expansionData.f32;
    float dynamicRange = getDynamicRange(cast_PhysicalToHeader(parent));
    float scaleFactor = parent->get_scale_factor(parent) * scaleAdjustment * (float)MAX_SINT16 / dynamicRange; // Scale to dynamic range, 16bit signed output
    SystemTime_t timestamp = self->timestamp;
    SInt32* sendata = data;
    SInt32 xi, yi, zi;
    float x, y, z;

    portDISABLE_INTERRUPTS();

    xi = sendata[0];
    yi = sendata[1];
    zi = sendata[2];

    portENABLE_INTERRUPTS();

    x = xi * scaleFactor;
    y = yi * scaleFactor;
    z = zi * scaleFactor;

    x = SATURATE(MAX_SINT16, x, MIN_SINT16);
    y = SATURATE(MAX_SINT16, y, MIN_SINT16);
    z = SATURATE(MAX_SINT16, z, MIN_SINT16);

    output.x = (SInt16)x;
    output.y = (SInt16)y;
    output.z = (SInt16)z;

    reportSensorEvent(self, &output, timestamp);

    return SensorOK;
}

VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor DESCRIPTOR_NAME = {
    .triggerSource = {
        .sensor = {
            .type = {
                .value = SENSOR_INPUT,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .physicalSource = {
        .sensor ={
            .type = {
                .value = SENSOR_INPUT,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        },
    },

    .info = {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type = {
        .value = SENSOR_TYPE_BSX(SENSOR_OUTPUT),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .expansionData = {
        .f32 = SCALE_FACTOR,
    },

    .priority = PRIORITY_2, // high priority

    .handle_sensor_data = handle_sensor_data,
    .outputPacketSize = sizeof(output_t),
}; //lint !e785
