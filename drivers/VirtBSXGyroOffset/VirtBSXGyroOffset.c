////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtBSXGyroOffset/VirtBSXGyroOffset.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual BSX gyroscope offset sensor.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2017-2018 EM Microelectronic
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
#include <bsx_support.h>
#include <bsx_vector_index_identifier.h>
#include <arc.h>
#include <hw_versions.h>

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV    (1u)

/*********************************************************************/
/* static variable */
static bool throw_data;
static output_3axis_t prev_data;

/*********************************************************************/
/* function definitions */

/*!
 * @brief This function force to report event.
 *
 * @param[in] self : Current virtual sensor descriptor
 * @param[in] mode : Sensor power mode
 *
 * @return None
 */
static void force_report_event(VirtualSensorDescriptor *self, SensorPowerMode mode)
{
    UNUSED(self);
    UNUSED(mode);
    throw_data = TRUE;
}

/*!
 * @brief This function handle sensor offset raw data.
 *
 * @param[in] self : Current virtual sensor descriptor
 * @param[in] data : Physical raw fata
 *
 * @return Sensor status
 */
static SensorStatus handle_offset_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    if (!self || !data)
    {
        return SensorUnknownError;
    }

    float scaleAdjustment = self->expansionData.f32;
    /* Scale to dynamic range, 16bit signed output */
    float scaleFactor = scaleAdjustment * (float)MAX_SINT16 / (float)getDynamicRange(cast_VirtualToHeader(self));
    const bsx_fifo_data_t* bsxdata = data;
    output_3axis_t output;

    for (bsx_vector_index_identifier_t i = BSX_ID_MOTION_VECTOR_X; i <= BSX_ID_MOTION_VECTOR_Z; i++)
    {
        /* Note: this is slightly slower than manual unrolling, but it does save a bit of space.*/
        float axis = bsxdata->content_p[i].sfp;
        axis = axis * scaleFactor;
        axis = SATURATE(MAX_SINT16, axis, MIN_SINT16);
        output.data[i] = (SInt16)axis;
    }

    if (throw_data || (output.x != prev_data.x) || (output.y != prev_data.y) || (output.z != prev_data.z))
    {
        throw_data = FALSE;
        prev_data.x = output.x;
        prev_data.y = output.y;
        prev_data.z = output.z;
        reportSensorEvent(self, &output, self->timestamp);
    }
    return SensorOK;
} //lint !e818

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtgyrooffset_descriptor = {
    .physicalSource =
    {
        .sensor =
        {
            .type =
            {
                .value = BSX_INPUT_ID_ANGULARRATE,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        }
    },

    .info =
    {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type =
    {
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_OFFSET),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
        .on_change = TRUE,
    }, //lint !e785

    .expansionData = {
        /* Scale factor to convert BSX4 output (rad/s) to HIF output (deg/s) */
        .f32 = (BSX_CONSTANT_UNIT_SCALING_RADIAN2DEGREE),
    },

    .outputPacketSize = sizeof(output_3axis_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = handle_offset_sensor_data,
    .mode_changed = force_report_event,
}; //lint !e785
