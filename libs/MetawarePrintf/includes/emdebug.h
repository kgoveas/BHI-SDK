////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/MetawarePrintf/includes/emdebug.h
///
/// @project    EM7189
///
/// @brief      Commands for registering debug values.
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

#ifndef EMDEBUG_H
#define EMDEBUG_H

#include <types.h>
#include <stdio.h>
#include <stdarg.h>
#include <hooks_support.h>

typedef size_t (*emwrite_handler_t)(const void *ptr, size_t elem, size_t count, void *fp, UInt8 packet_header);

typedef struct emwrite_t {
    emwrite_handler_t   handler;
    struct emwrite_t*   next;
} emwrite_t;

void registerEMWriteHandler(emwrite_t* structure, emwrite_handler_t handler);

void registerPrintfHandler(int (*handler)(const char* format, va_list args));

// To use printf, this hook must be referenced using HOOK_REF.
HOOK_DEF(initOnce, hookInitPrintf, HOOK_PRIORITY_ROM, void RECLAIM_TEXT);

#endif /* !EMDEBUG_H */
