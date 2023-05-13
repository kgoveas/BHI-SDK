////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Bootloader/includes/context.h
///
/// @project    EM7189
///
/// @brief
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
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
#ifndef CONTEXT_H
#define CONTEXT_H

#include <types.h>


typedef struct {
    union {
        UInt32 reg[32];
        struct {
            UInt32 r0;
            UInt32 r1;
            UInt32 r2;
            UInt32 r3;
            UInt32 r4;
            UInt32 r5;
            UInt32 r6;
            UInt32 r7;
            UInt32 r8;
            UInt32 r9;
            UInt32 r10;
            UInt32 r11;
            UInt32 r12;
            UInt32 r13;
            UInt32 r14;
            UInt32 r15;
            UInt32 r16;
            UInt32 r17;
            UInt32 r18;
            UInt32 r19;
            UInt32 r20;
            UInt32 r21;
            UInt32 r22;
            UInt32 r23;
            UInt32 r24;
            UInt32 r25;
            UInt32 gp;
            UInt32 fp;
            UInt32 sp;
            UInt32 ilink;
            UInt32 r30;
            UInt32 blink;
        } names;
    } r;
    UInt32 pc;
} em_context_t;

void save_context(em_context_t* context);

#endif /* CONTEXT_H */
