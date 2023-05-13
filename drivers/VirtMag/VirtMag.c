////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtMag/VirtMag.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual mag.
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
#include <arc.h>
#include <FreeRTOS.h>
#include "VirtMag.h"

/*********************************************************************/
/* local macro definition */
#define DRIVER_REV      (3u)

/*********************************************************************/
/* functions */

/*!
 * @brief This function reports sensor data to host
 *
 * @param[in] self : Descriptor for virtual sensor
 * @param[in] data : physical raw data
 *
 * @return  SensorOK for sucessful data report. Other values for failure.
 */
static SensorStatus mag_handle_sensor_data(VirtualSensorDescriptor* self, void* data)
{
    output_t output;
    PhysicalSensorDescriptor* parent = cast_HeaderToPhysical(getSensorParent(cast_VirtualToHeader(self)));

    float scaleAdjustment = self->expansionData.f32;
    float dynamicRange = getDynamicRange(cast_PhysicalToHeader(parent));
    float scaleFactor = parent->get_scale_factor(parent) * scaleAdjustment * (float)MAX_SINT16 /
                        dynamicRange; // Scale to dynamic range, 16bit signed output
    SystemTime_t timestamp = self->timestamp;
    int32_t *sendata = data;
    int32_t xi, yi, zi;
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

    output.x = (int16_t)x;
    output.y = (int16_t)y;
    output.z = (int16_t)z;

    reportSensorEvent(self, &output, timestamp);

    return SensorOK;
}

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtmag_descriptor = {
    .triggerSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_MAGNETICFIELD,
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
                .value = BSX_INPUT_ID_MAGNETICFIELD,
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
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_RAW),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .expansionData =
    {
        .f32 = 0.1F, /* Convert from 0.1uT to uT) */
    },

    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = mag_handle_sensor_data,
    .outputPacketSize = sizeof(output_t),
};  //lint !e785
