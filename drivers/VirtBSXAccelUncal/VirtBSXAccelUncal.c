////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtBSXAccelUncal/VirtBSXAccelUncal.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual BSX accelerometer uncalibrated
///             sensor.
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
#include <bsx_support.h>
#include <arc.h>
#include <hw_versions.h>

/*********************************************************************/
/* local macro definition */
#define DRIVER_REV      (1u)

/*********************************************************************/
/* virtual sensor descriptor */
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtacceluncal_descriptor = {
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
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_RAW),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .expansionData =
    {
        /* Scale factor to convert BSX4 output (mps2) to HIF output (g) */
        .f32 = (1.0F / BSX_CONSTANT_GRAVITY_STANDARD),
    },

    .maxRate = 800.0F,
    .minRate = 1.5625F,

    .outputPacketSize = sizeof(output_3axis_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = BSXSupport_handle_3axis_sensor_data,
}; //lint !e785
