////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtGyro/VirtGyro.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual gyro.
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

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <host.h>
#include <bsx_constant.h>
#include <arc.h>
#include <FreeRTOS.h>
#include "VirtGyro.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV    (3u)

/*********************************************************************/
/* function definitions */

/*!
 * @brief This function handle sensor physical raw data.
 *
 * @param[in] self : Current virtual sensor descriptor
 * @param[in] data : Physical raw fata
 *
 * @return Sensor status
 */
static SensorStatus gyro_handle_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    output_t output;
    PhysicalSensorDescriptor *parent = cast_HeaderToPhysical(getSensorParent(cast_VirtualToHeader(self)));

    float scale_adjustment = self->expansionData.f32;
    float dynamic_range = getDynamicRange(cast_PhysicalToHeader(parent));
    /* Scale to dynamic range, 16bit signed output */
    float scale_factor = parent->get_scale_factor(parent) * scale_adjustment * (float)MAX_SINT16 / dynamic_range;
    SystemTime_t timestamp = self->timestamp;
    int32_t *sendata = data;
    int32_t xi, yi, zi;
    float x, y, z;

    portDISABLE_INTERRUPTS();

    xi = sendata[0];
    yi = sendata[1];
    zi = sendata[2];

    portENABLE_INTERRUPTS();

    x = xi * scale_factor;
    y = yi * scale_factor;
    z = zi * scale_factor;

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
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtgyro_descriptor = {
    .triggerSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_ANGULARRATE,
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
                .value = BSX_INPUT_ID_ANGULARRATE,
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
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_RAW),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .expansionData =
    {
        .f32 = (1.0F / BSX_CONSTANT_UNIT_SCALING_RADIAN2DEGREE),
    },

    .priority = PRIORITY_2, /* high priority */

    .handle_sensor_data = gyro_handle_sensor_data,
    .outputPacketSize = sizeof(output_t),
}; //lint !e785
