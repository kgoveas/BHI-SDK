////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/BSXSupport/includes/bsx_mt_support.h
///
/// @project    EM7189
///
/// @brief      BSX Multithreading Support Interface.
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

#ifndef _BSX_MT_SUPPORT_H_
#define _BSX_MT_SUPPORT_H_

#include <bsx_library.h>
#include <bsx_return_value_identifier.h>
#include <bsx_datatypes.h>
#include <bsx_constant.h>
#include <hooks_support.h>
typedef struct bsxs_input_data_t bsxs_input_data_t;

#define ENABLE_MULTITHREADING_CALIBRATION
#if 1 // (ROM_VERSION != ROM_7189_DI01)
#define MERGE_VIRTBSX_WITH_MAINBSX // uncomment to eliminate main BSX task and run inside virt sensor task for VirtBSX
#endif

extern void BSXMT_initialize_tasks(VirtualSensorDescriptor* self);
extern uint32_t BSXMT_wait_for_trigger(void);
extern uint32_t BSXMT_wait_for_calib_trigger(void);
extern void BSXMT_calibration_complete(void);
extern void BSXMT_get_inputs(bsxs_input_data_t *input_package);
extern void BSXMT_trigger_main_task(void);
extern void BSXMT_clear_triggers(void);

#endif
