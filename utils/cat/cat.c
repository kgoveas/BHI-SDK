////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/cat/cat.c
///
/// @project    EM7189
///
/// @brief      Utility to join multiple binary files together.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE (4u * 1024u)
int main(int argc, char const *argv[])
{
    int total_size = 0;
    int i;
    int pad     = 0;
    int arg_idx = 1;
    if (argc > 1)
    {
        if (memcmp(argv[arg_idx], "-p", 2) == 0)
        {
            pad = 1;
            arg_idx++;
        }
        FILE *out = fopen(argv[arg_idx++], "wb");
        if (!out)
        {
            fprintf(stderr, "Unable to create output file '%s'\n", argv[1]);
            exit(-1);
        }

        for (i = arg_idx; i < argc; i++)
        {
            FILE *in = fopen(argv[i], "rb");
            if (!in)
            {
                fprintf(stderr, "Unable to open input file '%s'\n", argv[i]);
                exit(-2);
            }
            uint8_t buffer[BUFFER_SIZE];
            size_t  read_size = 0;
            do
            {
                read_size = fread(buffer, 1, sizeof(buffer), in);
                if (read_size > 0)
                {
                    fwrite(buffer, 1, read_size, out);
                    total_size += read_size;
                }
            } while (read_size == sizeof(buffer));

            fclose(in);
        }

        if (pad && (((total_size - 124) % 256) == 8))
        {
            uint8_t pad[4];
            // HW workaround - pad this file by 4 bytes of 0xff
            memset(pad, 0xff, 4);
            fwrite(pad, 1, 4, out);
        }
        fclose(out);
    }
    else
    {
        fprintf(stderr, "USAGE: %s [-p] file1 file2 ...'\n", argv[0]);
        exit(-1);
    }

    return 0;
}
