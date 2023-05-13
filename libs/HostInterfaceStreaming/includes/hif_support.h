////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/hif_support.h
///
/// @project    EM7189
///
/// @brief      Support header for host interface
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

#ifndef HIF_SUPPORT_H
#define HIF_SUPPORT_H

#if _ARCVER /* Only use for arc build */

#include <types.h>
#include <arc.h>


extern volatile SystemTime_t gHostIRQTimestamp;
extern volatile SystemTime_t gHostEVTimestamp;

/**
 * \brief place for main host interface (in program RAM or Flash) to intercept the host register read IRQ; note: this only occurs for registers which are read-irq-enabled
 * \param addr - the register address the host read
 * \param data - the data the host read
 */
typedef void (* HIF_READ_CALLBACK)(UInt8 addr, UInt8 data);

/**
 * \brief host register write IRQ; note: this only occurs for registers which are write-irq-enabled
 * \param addr - the register address the host wrote to
 */
typedef void (* HIF_WRITE_CALLBACK)(UInt8 addr, UInt8 data);

extern HIF_READ_CALLBACK gHIFReadCallback;
extern HIF_WRITE_CALLBACK gHIFWriteCallback;

#endif /* _ARCVER */
#endif /* !HIF_SUPPORT_H */
