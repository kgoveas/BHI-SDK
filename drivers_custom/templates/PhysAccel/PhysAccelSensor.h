////////////////////////////////////////////////////////////////////////////////
///
/// @file       drivers_custom/templates/PhysAccel/PhysAccelSensor.h
///
/// @project    EM7189
///
/// @brief      Template driver for non-composite physical
///             sensor.
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

#ifndef PHYS_ACCEL_H_
#define PHYS_ACCEL_H_

/*****************************/
/*  Definitions and types    */
/*****************************/

#define DEFINE_ME 0

#define PHYS_ACCEL_SAMPLE_DATA_SIZE  (6)

#define PHYS_ACCEL_CHIP_ID (DEFINE_ME)

#define PHYS_ACCEL_CHIP_ID_ADDR   (DEFINE_ME)
#define PHYS_ACCEL_USER_DATA_ADDR (DEFINE_ME)

#define PHYS_ACCEL_RANGE_2G  (DEFINE_ME)
#define PHYS_ACCEL_RANGE_4G  (DEFINE_ME)
#define PHYS_ACCEL_RANGE_8G  (DEFINE_ME)
#define PHYS_ACCEL_RANGE_16G (DEFINE_ME)

#define PHYS_ACCEL_MAX_DYNAMIC_RANGE (16)
#define PHYS_ACCEL_MAX_ODR (400.0F)
#define PHYS_ACCEL_MIN_ODR (12.5F)
#define PHYS_ACCEL_MAX_CURRENT (0.160F)


/*****************************/
/*  Extern declarations      */
/*****************************/

/*****************************/
/*  Function prototypes      */
/*****************************/

#endif
