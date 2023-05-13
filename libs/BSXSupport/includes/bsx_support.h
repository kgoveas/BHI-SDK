////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/BSXSupport/includes/bsx_support.h
///
/// @project    EM7189
///
/// @brief      BSX Support Interface.
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
#include <bsx_library.h>
#include <bsx_return_value_identifier.h>
#include <bsx_datatypes.h>
#include <bsx_constant.h>
#include <hooks_support.h>
#include <SensorAPI.h>

#define ENABLE_MULTITHREADING_CALIBRATION

#define BSX_DIMENSION_INPUT_MOTION_VECTOR             (BSX_DIMENSION_MOTION_VECTOR)
typedef struct bsxs_input_data_t
{
    SInt32 data[4];
    TimestampTime_t timestamp;
    UInt8 sensor_type;
    float scale_factor;
} bsxs_input_data_t;

typedef union {
    SInt16 data[3];
    struct {
        SInt16 x;
        SInt16 y;
        SInt16 z;
    };
} output_3axis_t;

typedef union {
    SInt16 data[5];
    struct {
        SInt16      x;
        SInt16      y;
        SInt16      z;
        SInt16      w;
        UInt16      accuracy;
    };
} output_3axis_rot_t;

typedef struct {
    SInt16      x; // heading
    SInt16      y; // pitch
    SInt16      z; // roll
} output_orientation_t;

typedef struct {
} output_gesture_t;

typedef struct {
    UInt8       activity_off;
    UInt8       activity_on;
} output_activity_t;

typedef struct {
    UInt32      count;
} output_step_count_t;

typedef struct {
    UInt8      orientation;
} output_device_orientation_t;

typedef union _logging_mode_
{
    struct
    {
        UInt8 log_bsx4_dosteps : 1;
        UInt8 log_bsx4_updatesub: 1;
        UInt8 log_bsx4_calib : 1;
        UInt8 log_bsx4_getoutsignal : 1;
        UInt8 log_bsx4_getoutsignal_all : 1;
        UInt8 log_bsx4_reserved : 3; // not used
    } bits;
    UInt8 val;
} BSX4_LOGGING_MODE;

#define ERROR_BSX_INIT          (ERROR_ALGO /*+ 0*/)
#define ERROR_BSX_DO_STEP       (ERROR_ALGO + 1)
#define ERROR_BSX_UPDATE_SUB    (ERROR_ALGO + 2)
#define ERROR_BSX_GET_SUB       (ERROR_ALGO + 3)
#define ERROR_BSX_GET_PHYS      (ERROR_ALGO + 4)
#define ERROR_BSX_SET_PHYS_RATE (ERROR_ALGO + 5)
#define ERROR_BSX_GET_SELF      (ERROR_ALGO + 6)
#define ERROR_BSX_INIT_GET_CHAR (ERROR_ALGO + 7)
#define ERROR_BSX_INIT_SET_CONF (ERROR_ALGO + 8)
#define ERROR_BSX_INIT_SENSOR_DISABLED  (ERROR_ALGO + 9)

extern bsx_instance_t gBSXInstance;
extern BSX4_LOGGING_MODE gBSXLoggingEnabled;

extern void JLI setBSXLoggingMode(UInt8 mode);

extern void BSXSupport_main_task(void *params);
extern void BSXSupport_calibration_task(void *params);

extern SensorStatus BSXSupport_initialize_sensor(VirtualSensorDescriptor* self);
extern void BSXSupport_process_samples(void);
extern void BSXSupport_convert_input_data(const bsxs_input_data_t *phys_input, bsx_fifo_data_x32_t *input, bsx_data_content_t *sample_data);


/* Main Thread */
extern SensorStatus BSXSupport_handle_sensor_data(VirtualSensorDescriptor* self, void* data);

/* 3 Axis */
extern SensorStatus BSXSupport_handle_3axis_sensor_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_handle_3axis_rotation_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_handle_orientation_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_handle_3axis_passthrough_data(VirtualSensorDescriptor* self, void* data);

/* Gesture */
extern SensorStatus BSXSupport_handle_gesture_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_handle_oneshot_gesture_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_report_last_gesture_data(VirtualSensorDescriptor* self);

/* Activity */
extern SensorStatus BSXSupport_handle_activity_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_report_last_activity_data(VirtualSensorDescriptor* self);

/* step counter */
extern SensorStatus BSXSupport_handle_step_count_data(VirtualSensorDescriptor* self, void* data);
extern SensorStatus BSXSupport_handle_last_step_count_data(VirtualSensorDescriptor* self);

/* Device Orientation */
SensorStatus BSXSupport_handle_device_orientation_data(VirtualSensorDescriptor* self, void* data);
SensorStatus BSXSupport_handle_last_device_orientation_data(VirtualSensorDescriptor* self);

/* Custom output Sensors */
SensorStatus BSXSupport_trigger_custom_sensors(VirtualSensorDescriptor* self, void* data);


#define ALGORITHM_PARAM_BSX_VERSION     (0x7E)
#define ALGORITHM_PARAM_SIC             (0x7D)
#define ALGORITHM_PARAM_CALIB_STATE_BASE (0x0)
#define ALGORITHM_PARAM_CALIB_STATE_MAX (0x40)

extern bool algorithmPageWriteHandler(UInt8 param, UInt16 length, UInt8 buffer[]);
extern bool algorithmPageReadHandler(UInt8 param, UInt16 length, UInt8 buffer[], UInt16 *ret_length);

float bsxRateToFrameworkRate(float bsx_rate);

bool BSXSupport_thread_triggered(void);

HOOK_DEF(initOnce, bsxInitOnceHookKernel, HOOK_PRIORITY_ROM, void);
HOOK_DEF(PhysicalRate, bsxHookPhysicalRate, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float* rate);
HOOK_DEF(TimerRate,    bsxHookTimerRate, HOOK_PRIORITY_ROM, void, VirtualSensorDescriptor* timer, float* rate);
HOOK_DEF(VirtualSensorsDetermined, bsxHookVirtualSensorsDetermined, HOOK_PRIORITY_ROM, void);
HOOK_DEF(determinePowerState, bsxDeterminePowerState, HOOK_PRIORITY_ROM, SensorPowerMode, PhysicalSensorDescriptor* phys);
HOOK_DEF(PhysicalRateChanged, bsxHookPhysicalRateChanged, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float rate);
