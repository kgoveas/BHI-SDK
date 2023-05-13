////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elf2bin/elf2bin.cpp
///
/// @project    EM7189
///
/// @brief      Tool to generate a binary (BIN) for a given
///             memory region.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2015-2018 EM Microelectronic
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

#define __STDC_FORMAT_MACROS 1

#include <elf_support.h>

#include <cstdio>

//#define DEBUG_PRINT (1)

#if WIN32
#include <Windows.h>
#include <io.h>

#define PRIx64 "I64x"

#define err(code, ...)                \
    do                                \
    {                                 \
        fprintf(stderr, __VA_ARGS__); \
        fprintf(stderr, "\n");        \
        exit(code);                   \
    } while (0)

#define errx(code, ...)               \
    do                                \
    {                                 \
        fprintf(stderr, __VA_ARGS__); \
        fprintf(stderr, "\n");        \
        exit(code);                   \
    } while (0)

#define sscanf sscanf_s

FILE *my_fopen(const char *filename, const char *mode)
{
    FILE *pFile = NULL;

    if (0 != fopen_s(&pFile, filename, mode))
        return NULL;

    return pFile;
}
#else
#include <err.h>
#include <inttypes.h>
#include <sys/queue.h>
#include <unistd.h>

#define my_fopen(filename, mode) fopen(filename, mode)
#endif // WIN32

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEX_PREFIX ("0x")

int usage(const char *argv[])
{
    fprintf(stderr,
            "Usage: %s ELF BIN [ADDR_START SIZE] [SYMBOL_START SYMBOL_END]\n"
            "\n"
            "ELF          Input ELF file.\n"
            "MIF          Output Memory Initialization File (MIF).\n"
            "ADDR_START   Start address of the memory region (in hex) to dump.\n"
            "SIZE         Size of the memory region to dump (in decimal bytes).\n"
            "SYMBOL_START Start address of the memory region (symbol) to dump.\n"
            "SYMBOL_END   End address of the (symbol) to dump.\n",
            argv[0]);

    return -1;
}

int WriteBuffer(uint8_t *pMemory, const char *pOutFile, uint32_t size)
{
    FILE *pOut;

    if (0 == pMemory)
    {
        return -1;
    }

    //
    // Dump the memory region to the output file.
    //
    if (NULL == (pOut = my_fopen(pOutFile, "wb")))
    {
        fprintf(stderr, "Filed to open file for writing: %s\n", pOutFile);

        return -1;
    }

    if (1 != fwrite(pMemory, size, 1, pOut))
    {
        fprintf(stderr, "Filed to write data to %s\n", pOutFile);

        return -1;
    }

    //
    // Close the elf file.
    //
    fclose(pOut);

    delete[] pMemory;

    return 0;
}

int ExtractSections(const char *pInFile, const char *pOutFile,
                    uint32_t startAddress, uint32_t endAddress, uint32_t size)
{
    // Pointer to the memory region to dump data into.
    uint8_t *pMemory;

    pMemory = ExtractSections(pInFile, startAddress, endAddress, size);
    return WriteBuffer(pMemory, pOutFile, size);
}

int ExtractSections(const char *pInFile, const char *pOutFile,
                    const char *pStartSymbol, const char *pEndSymbol)
{
    // Pointer to the memory region to dump data into.
    uint8_t *pMemory;
    uint32_t size = 0;

    pMemory = ExtractSections(pInFile, pStartSymbol, pEndSymbol, &size);
    return WriteBuffer(pMemory, pOutFile, size);
}


int ExtractAllSections(const char *pInFile, const char *pOutFile)
{
    // Pointer to the memory region to dump data into.
    uint8_t *pMemory;
    uint32_t size;

    pMemory = ExtractAllSections(pInFile, &size);
    return WriteBuffer(pMemory, pOutFile, size);
}

int main(int argc, const char *argv[])
{
    const char *pStartSymbol = 0;
    const char *pEndSymbol   = 0;

    uint32_t startAddress = 0;
    uint32_t endAddress   = 0;
    uint32_t size         = 0;

    const char *pInFile;
    const char *pOutFile;

    if (3 != argc && 5 != argc)
    {
        return usage(argv);
    }

    pInFile  = argv[1];
    pOutFile = argv[2];

    if (5 == argc)
    {
        if (strlen(HEX_PREFIX) <= strlen(argv[3]) && 0 == memcmp(
                                                              HEX_PREFIX, argv[3], strlen(HEX_PREFIX)))
        {
            if (1 != sscanf(&argv[3][strlen(HEX_PREFIX)], "%x", &startAddress))
            {
                pStartSymbol = argv[3];
            }
        }
        else
        {
            if (1 != sscanf(argv[3], "%x", &startAddress))
            {
                pStartSymbol = argv[3];
            }
        }

        if (1 != sscanf(argv[4], "%u", &size))
        {
            pEndSymbol = argv[4];


#ifdef DEBUG_PRINT
            printf("Input file:    %s\n", pInFile);
            printf("Output file:   %s\n", pOutFile);
            printf("Start Symbol:  %s\n", pStartSymbol);
            printf("End Symbol:    %s\n", pEndSymbol);
            printf("Size:          %lu\n", size);
#endif // DEBUG_PRINT

            return ExtractSections(pInFile, pOutFile, pStartSymbol, pEndSymbol);
        }
        else
        {
            endAddress = startAddress + size;


#ifdef DEBUG_PRINT
            printf("Input file:    %s\n", pInFile);
            printf("Output file:   %s\n", pOutFile);
            printf("Start Address: 0x%08x\n", startAddress);
            printf("End Address:   0x%08x\n", endAddress);
            printf("Size:          %lu\n", size);
#endif // DEBUG_PRINT

            return ExtractSections(pInFile, pOutFile, startAddress, endAddress,
                                   size);
        }
    }
    else
    {
        return ExtractAllSections(pInFile, pOutFile);
    }
}
