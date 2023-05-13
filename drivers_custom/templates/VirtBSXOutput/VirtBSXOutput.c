////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/templates/VirtBSXOutput/VirtBSXOutput.c
///
/// @project    EM7189
///
/// @brief      Template driver used to report bsx output data to the host.
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

/*********************************************************************/
/* system header files */
#include <SensorAPI.h>
#include <host.h>
#include <bsx_support.h>
#include <bsx_vector_index_identifier.h>
#include <arc.h>
#include <hw_versions.h>

/*********************************************************************/
/* local macro definition */
#define DRIVER_REV          (1u)
#define SENSOR_INPUT        BSX_INPUT_ID_ANGULARRATE            /* Physical sensor to use for looking up items such as power consumption, dynamic range, etc */
#define BSX_OUTPUT          BSX_OUTPUT_ID_ANGULARRATE_OFFSET    /* BSX output */
/* #define BSX_SCALE        (1.0F / BSX_CONSTANT_GRAVITY_STANDARD) */ /* scale factor for accel based sensors to convert from m/s2 to g */
/* #define BSX_SCALE        (1.0f) */ /* scale factor for mag based sensors */
#define BSX_SCALE           (BSX_CONSTANT_UNIT_SCALING_RADIAN2DEGREE) /* scale factor for gyro based sensors to convert from radians to degrees */
#define ON_CHANGE_SENSOR    (1) /* If set, causes the handle sensor data function to only report data if it has change or if the sensor was turned on. */

/*********************************************************************/
/* static variables */
#if ON_CHANGE_SENSOR
static bool throw_data;
static output_3axis_t prev_data;
static SystemTime_t timestamp = 0;
static output_3axis_t output;
#endif

/*********************************************************************/
/* functions */

#if ON_CHANGE_SENSOR
static void force_report_event(VirtualSensorDescriptor* self, SensorPowerMode mode)
{
    /* Any time the power mode changes for an on change sensor.
     we need to report new sample data.
    Setting throw_data to true here results in the handle_sensor_data function reporting new data even if it is the same as before.
    */
    UNUSED(self);
    UNUSED(mode);
    throw_data = TRUE;
}
#endif

/*!
 * @brief This function reports sensor data to host
 *
 * @param[in] self : Descriptor for virtual sensor
 * @param[in] data : physical raw data
 *
 * @return  SensorOK for sucessful data report. Other values for failure.
 */
static SensorStatus handle_sensor_data(VirtualSensorDescriptor *self, void *data)
{
    if(!self || !data) return SensorUnknownError;

    float scaleAdjustment = self->expansionData.f32;
    float scaleFactor = scaleAdjustment * (float)MAX_SINT16 / (float)getDynamicRange(cast_VirtualToHeader(self)); /* Scale to dynamic range, 16bit signed output */
    const bsx_fifo_data_t* bsxdata = data;

    for(bsx_vector_index_identifier_t i = BSX_ID_MOTION_VECTOR_X; i <= BSX_ID_MOTION_VECTOR_Z; i++)
    {
        /* Scale the sensor data to the output format needed */
        float axis = bsxdata->content_p[i].sfp;
        axis = axis * scaleFactor;
        axis = SATURATE(MAX_SINT16, axis, MIN_SINT16);
        output.data[i] = (SInt16)axis;
    }

#if ON_CHANGE_SENSOR
    if (throw_data || (output.x != prev_data.x) || (output.y != prev_data.y) || (output.z != prev_data.z))
    {
        throw_data = FALSE;
        prev_data.x = output.x;
        prev_data.y = output.y;
        prev_data.z = output.z;
        reportSensorEvent(self, &output, self->timestamp);
        timestamp = self->timestamp;
    }
#else
    reportSensorEvent(self, &output, self->timestamp);
#endif

    return SensorOK;
} //lint !e818

/*!
 * @brief Virtual sensor processing callback for last sensor data
 *
 * @param[in] self : Descriptor for virtual sensor
 *
 * @return SensorOK for sucessful data report. Other values for failure.
 */
SensorStatus get_last_sensor_data(VirtualSensorDescriptor *self)
{
#if ON_CHANGE_SENSOR
    reportSensorEvent(self, &output, timestamp);
#else
    reportSensorEvent(self, &output, self->timestamp);
#endif

    return SensorOK;
}

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor DESCRIPTOR_VIRT_OUTPUT_NAME = {
    .physicalSource = {
        .sensor = {
            .type = {
                .value = SENSOR_INPUT,
                .flags = DRIVER_TYPE_PHYSICAL_FLAG,
            }, //lint !e785
        }
    },

    .info = {
        .id = DRIVER_ID,
        .version = DRIVER_REV,
    }, //lint !e785

    .type = {
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
        .on_change = ON_CHANGE_SENSOR,
    }, //lint !e785

    .expansionData = {
        /* Scale factor to convert BSX4 output. example: (rad/s) to HIF output (deg/s) */
        .f32 = BSX_SCALE,
    },

    .outputPacketSize = sizeof(output_3axis_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = handle_sensor_data,
    .get_last_sensor_data = get_last_sensor_data,
#if ON_CHANGE_SENSOR
    .mode_changed = force_report_event,
#else
    .mode_changed = NULL,
#endif
}; //lint !e785
