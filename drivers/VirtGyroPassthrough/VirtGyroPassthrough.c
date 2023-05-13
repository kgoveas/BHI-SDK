////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers/VirtGyroPassthrough/VirtGyroPassthrough.c
///
/// @project    EM7189
///
/// @brief      Driver for the virtual Gyroerometer.
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
#include <FreeRTOS.h>
#include "VirtGyroPassthrough.h"

/*********************************************************************/
/* local macro definitions */
#define DRIVER_REV    (1u)

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
static SensorStatus gyro_passthrough_handle_sensor_data(VirtualSensorDescriptor *self, void *data)
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
VIRTUAL_SENSOR_DESCRIPTOR VirtualSensorDescriptor virtGyro_calc_descriptor =
{
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
        .sensor = {
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
        .value = SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_PASSTHROUGH),
        .flags = DRIVER_TYPE_VIRTUAL_FLAG,
    }, //lint !e785

    .outputPacketSize = sizeof(output_t),
    .priority = PRIORITY_2, /* high priority */

    .initialize = NULL,
    .handle_sensor_data = gyro_passthrough_handle_sensor_data,
    .mode_changed = NULL,
}; //lint !e785
