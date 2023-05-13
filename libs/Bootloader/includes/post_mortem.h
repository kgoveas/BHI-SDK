////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Bootloader/includes/post_mortem.h
///
/// @project    EM7189
///
/// @brief      Post mortem code for handling watchdog timeouts and
///             unhandled exceptions
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
#ifndef POST_MORTEM_H
#define POST_MORTEM_H

#include <context.h>

typedef struct _post_mortem_info
{
    em_context_t em_ctx;    // 0x00 - 0x83
    UInt8  valid;           // 0x84
    UInt8  flags;           // 0x85
    UInt16 stackSize;       // 0x86
    UInt32 stackStart;      // 0x88
    UInt32 eret;            // 0x8c
    UInt32 erbta;           // 0x90
    UInt32 erstatus;        // 0x94
    UInt32 ecr;             // 0x98
    UInt32 efa;             // 0x9c
    UInt32 diagnostic;      // 0xa0
    UInt32 icause;          // 0xa4
    UInt8  debugValue;      // 0xa8
    UInt8  debugState;      // 0xa9
    UInt8  errorReason;     // 0xaa
    UInt8  interruptState;  // 0xab
    UInt8  hostIntCtrl;     // 0xac
    UInt8  resetReason;     // 0xad
    UInt8  hostResetCount;  // 0xae
    UInt8  errorReport;     // 0xaf
    UInt32 mpu_ecr;         // 0xb0
    UInt32 user_sp;         // 0xb4
    UInt8  reserved[12];    // 0xb8
    UInt32 stackCrc;        // 0xc4
    UInt32 crc;             // 0xc8
    // UInt8 stack[0]; // Stack is copied to the end of this structure
} post_mortem_info;

#define PM_FLAG_HOST_INT_CTRL_VALID     0x01
#define PM_FLAG_STACK_INFO_VALID        0x02
#define PM_FLAG_STACK_CONTENTS_INCLUDED 0x04
#define PM_FLAG_NO_AUTOEXEC             0x08

#define PM_ERR_TYPE_WDRESET         0x01
#define PM_ERR_INSTRUCTION_ERROR    0x02
#define PM_ERR_TYPE_MACHINE_CHECK   0x03
#define PM_ERR_MPU_LOW_ADDR         0x04
#define PM_ERR_MPU_HIGH_ADDR        0x05

#define PM_ERR_FLAG_NO_AUTOEXEC     0x01
#define PM_ERR_FLAG_MPU_READ        0x02

void PM_invalidate(void);
void PM_setStack(UInt32 start, UInt32 end);
bool PM_noAutoExec(void);
bool PM_isResetCause(UInt8 cause);
void PM_reportExceptionToHost(void);
void PM_getReport(UInt32 **dataPtr, UInt16 *dataSize);
void PM_setHostIntCtrl(UInt8 val);
void PM_triggerError(UInt8 error, UInt8 flags);
void __attribute__((noreturn)) PM_fatalError(UInt8 error, bool noAutoExec);

#endif /* !POST_MORTEM_H */
