////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/templates/VirtBSXCustomSource/VirtBSXCustomSource.c
///
/// @project    EM7189
///
/// @brief      Template driver used to consume bsx output data for custom algo.
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
#include <bsx_support.h>

#include <arc.h>
#include <hw_versions.h>

#define DRIVER_REV          (1u)

#define SENSOR_INPUT        BSX_INPUT_ID_ACCELERATION               /* Physical sensor to use for looking up items such as power consumption, dynamic range, etc */
#define BSX_OUTPUT          BSX_CUSTOM_ID_ACCELERATION_CORRECTED    /* BSX output */
#define BSX_SCALE           (1.0F / BSX_CONSTANT_GRAVITY_STANDARD) /* scale factor for accel based sensors to convert from m/s2 to g */
// #define BSX_SCALE           (1.0f) /* scale factor for mag based sensors */
// #define BSX_SCALE           (BSX_CONSTANT_UNIT_SCALING_RADIAN2DEGREE) /* scale factor for gyro based sensors to convert from radians to degrees */

/*****************************/
/*        Global Variables   */
/*****************************/

VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor DESCRIPTOR_VIRT_NAME = {
    .physicalSource = {
        .sensor = {
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
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
        .hidden = TRUE, /* This sensor should not be configured by the host directly */
    }, //lint !e785

    .expansionData = {
        // Scale factor to convert BSX4 output (mps2) to HIF output (g)
        .f32 = BSX_SCALE,
    },

    .maxRate = 800.0F,
    .minRate = 1.5625F,

    .outputPacketSize = sizeof(output_3axis_t),
    .priority = PRIORITY_2, // high priority

    .initialize = NULL,
    .handle_sensor_data = BSXSupport_trigger_custom_sensors,
}; //lint !e785
