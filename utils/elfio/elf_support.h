////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/elf_support.h
///
/// @project    EM7189
///
/// @brief      Support functions to ELF tasks.
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

#ifndef ELF_SUPPORT_H
#define ELF_SUPPORT_H

#include <stdint.h>
#include <elf_support.hpp>

#ifdef __cplusplus
extern "C" {
#endif

#define INVALID_ADDRESS     ((uint64_t)-1)

void* OpenElf(const char* filename);
bool SaveElf(void* elf, const char* filename);
void CloseElf(void* elf);

uint64_t get_section_size(void* elf, const char* section_name);
uint64_t get_section_addr(void* elf, const char* section_name);
const char* get_section_data(void* elf, const char* section_name);
bool set_section_data(void* elf, const char* section_name, const char* data, uint64_t size);

uint64_t get_symbol_addr(void* elf, const char* symbol_name);

#ifdef __cplusplus
}
#endif
#endif // ELF_SUPPORT_H
