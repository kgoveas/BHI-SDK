////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Bootloader/includes/bootloader_commands.h
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
#ifndef BOOTLOADER_COMMANDS_H
#define BOOTLOADER_COMMANDS_H

#include <command_processor.h>
#include <host_channels.h>

void erase_flash_start(COMMAND_HEADER *cmd, CHANNEL_DATA *cd);
void erase_flash_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void write_flash_start(COMMAND_HEADER *cmd, CHANNEL_DATA *cd);
void write_flash_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

bool boot_flash(bool isAutoExec);
void boot_flash_start(COMMAND_HEADER *cmd, CHANNEL_DATA *cd);
void boot_flash_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void post_mortem_data_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void upload_program_ram_start(COMMAND_HEADER *cmd, CHANNEL_DATA *cd);
void upload_program_ram_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);
void firmware_upload_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);
void firmware_upload_process(UInt8 *addr, UInt32 len, bool xferComplete);

void boot_program_ram_start(COMMAND_HEADER *cmd, CHANNEL_DATA *cd);
void boot_program_ram_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void trigger_error_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void read_otp_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

void test_support_main(COMMAND_HEADER *cmd, STATUS_HEADER *stat);

//lint -function( fopen(1), sendErrStatus(2) ) arg cannot be null
void sendErrStatus(UInt16 cmd, STATUS_HEADER *stat, UInt8 error);

#endif /* !BOOTLOADER_COMMANDS_H */
