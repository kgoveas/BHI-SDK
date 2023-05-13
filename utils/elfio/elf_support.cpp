////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/elf_support.cpp
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

#include <elfio/elfio2.hpp>
#include "elf_support.h"
#include "InitData.h"

#define STR_INITDAT                     ".initdat"
#define FILL_BYTE (0x00)

//#define DEBUG_PRINT (1)

using namespace ELFIO;


void* OpenElf(const char* filename)
{
    elfio2* pElf = new elfio2;

    if(!pElf->load(filename))
    {
        delete pElf;
        return NULL;
    }

    return (void*)pElf;
}

bool SaveElf(void* elf, const char* filename)
{
    elfio2* pElf = (elfio2*)elf;
    return pElf->save(filename);
}

void CloseElf(void* elf)
{
    elfio2* pElf = (elfio2*)elf;

    delete pElf;
}

uint8_t* ExtractSections(void* elf, uint32_t startAddress,
    uint32_t endAddress, uint32_t size)
{
    // Pointer to the memory region to dump data into.
    elfio2* pElf = (elfio2*)elf;
    uint8_t *pMemory = 0;

    uint32_t sectionStartAddress;
    uint32_t sectionEndAddress;
    uint32_t sectionSize;

    //
    // Allocate and fill the memory region.
    //
    pMemory = new uint8_t[size];
    memset(pMemory, FILL_BYTE, size);

    //
    // Dump all sections (at least partially) within the memory region.
    //
    for(std::vector<section*>::const_iterator it = pElf->sections.begin();
        it != pElf->sections.end(); ++it)
    {
        const section *pSection = *it;

        sectionStartAddress = (uint32_t)pSection->get_address();
        sectionSize = (uint32_t)pSection->get_size();
        sectionEndAddress = sectionStartAddress + sectionSize;

        if(SHT_PROGBITS == pSection->get_type() &&
            0 != (SHF_ALLOC & pSection->get_flags()) &&
            ((sectionEndAddress > startAddress && sectionEndAddress <= endAddress) ||       // Ends within the region.
            (sectionStartAddress >= startAddress && sectionStartAddress < endAddress) ||    // Starts within the region.
            (sectionStartAddress <= startAddress && sectionEndAddress >= endAddress))       // Completely contains the region.
            )
        {
            bool truncating = false;
            //
            // At least 1 byte from this section is within the memory region.
            //
            uint32_t sourceOffset;
            uint32_t destOffset;
            if(sectionStartAddress >= startAddress)
            {
                destOffset = sectionStartAddress - startAddress;
                sourceOffset = 0;
            }
            else
            {
                destOffset = 0;
                sourceOffset = startAddress - sectionStartAddress;
                sectionSize -= sourceOffset;
            }

            if(sectionEndAddress > endAddress)
            {
                truncating = true;
                sectionSize -= (sectionEndAddress - endAddress);
            }
            // Check if the end address of the section is within the memory
            // region of if the region should be clipped.

#ifdef DEBUG_PRINT
            printf("Copying %s: from 0x%08x to 0x%08x (0x%08x bytes)\n",
                pSection->get_name().c_str(),
                sourceOffset, destOffset, sectionSize);

            printf("Section %s: 0x%08x-0x%08x %s(%u bytes)\n",
                pSection->get_name().c_str(),
                sectionStartAddress, sectionEndAddress, (truncating || sourceOffset)? "(truncated) " : "", sectionSize);
#endif // DEBUG_PRINT

            // Read the section data.
            memcpy(&pMemory[destOffset], &pSection->get_data()[sourceOffset], sectionSize);
        }
    }

    return pMemory;
}

uint8_t* ExtractSections(const char *pInFile, const char* pStartSymbol,
    const char* pEndSymbol, uint32_t* size)
{

    // Create the elfio reader.
    elfio2 elf;

    //
    // Open the input ELF file.
    //
    if(!elf.load(pInFile))
    {
        return 0;
    }

    symbol* start_symbol = elf.symbols[pStartSymbol];
    symbol* end_symbol = elf.symbols[pEndSymbol];

    if(!start_symbol)
    {
        fprintf(stderr, "Error: unable to locate symbol '%s'\n", pStartSymbol);
        exit(-1);
    }

    if(!end_symbol)
    {
        fprintf(stderr, "Error: unable to locate symbol '%s'\n", pEndSymbol);
        exit(-1);
    }

    if(!size)
    {
        return 0;
    }

    uint32_t start_addr = (uint32_t)start_symbol->get_value();
    uint32_t end_addr = (uint32_t)end_symbol->get_value() - 1;
    if(end_addr < start_addr)
    {
        fprintf(stderr, "Error: unable to dump from  0x%08x to 0x%08x\n", start_addr, end_addr);
        exit(-1);
    }

    *size = end_addr - start_addr + 1;
    return ExtractSections((void*)&elf, start_addr, end_addr, *size);
}


uint8_t* ExtractSections(const char *pInFile, uint32_t startAddress,
    uint32_t endAddress, uint32_t size)
{
    // Create the elfio reader.
    elfio2 elf;

    //
    // Open the input ELF file.
    //
    if(!elf.load(pInFile))
    {
        return 0;
    }

    return ExtractSections((void*)&elf, startAddress, endAddress, size);
}

uint8_t* ExtractAllSections(const char *pInFile, uint32_t* pSize)
{
    bool foundStart = false;
    uint32_t startAddress = 0;
    uint32_t endAddress = 0;
    // Create the elfio reader.
    elfio2 elf;

    // Pointer to the memory region to dump data into.
    uint8_t *pMemory = 0;

    if(pSize)
    {
        *pSize = 0;
    }
    else
    {
        return 0;
    }

    //
    // Open the input ELF file.
    //
    if(!elf.load(pInFile))
    {
        return 0;
    }

    //
    // Determine the actual size of teh buffer needed.
    //
    for(std::vector<section*>::const_iterator it = elf.sections.begin();
        it != elf.sections.end(); ++it)
    {
        const section *pSection = *it;

        if(SHT_PROGBITS == pSection->get_type() &&
            0 != (SHF_ALLOC & pSection->get_flags()))
        {
            if(!foundStart)
            {
                foundStart = true;
                startAddress = (uint32_t)pSection->get_address();
            }
            else
            {
                if((uint32_t)pSection->get_address() > endAddress)
                {
                    uint32_t bytes = (uint32_t)pSection->get_address() - endAddress;
                    if(bytes < 8)
                    {
                        // Don't throw and error for expected alignment.
                    }
                    else if(bytes > 1024 * 1024)
                    {
                        printf("Error: inserting %u bytes of padding before section %s\n", bytes, pSection->get_name().c_str());
                        exit(-1);
                    }
                    else
                    {
                        /* Between 8 and 128 bytes of padding. */
                        printf("Warning: inserting %u bytes of padding before section %s\n", bytes, pSection->get_name().c_str());
                    }
                }
                else if((uint32_t)pSection->get_address() != endAddress)
                {
                    printf("Warning: inserting %u bytes in a possibly already allocated region for section %s\n", (uint32_t)pSection->get_size(), pSection->get_name().c_str());
                }
            }

            uint32_t currentEndAddress = (uint32_t)pSection->get_address() + (uint32_t)pSection->get_size();
            if(currentEndAddress > endAddress) endAddress = currentEndAddress;
        }
    }

    *pSize = endAddress - startAddress;

    if(!*pSize)
    {
        return 0;
    }

#ifdef DEBUG_PRINT
    printf("Allocating %d bytes for the output.\n", *pSize);
#endif
    //
    // Allocate and fill the memory region.
    //
    pMemory = new uint8_t[*pSize];
    memset(pMemory, FILL_BYTE, *pSize);
    //
    // Dump all sections (at least partially) within the memory region.
    //
    for(std::vector<section*>::const_iterator it = elf.sections.begin();
        it != elf.sections.end(); ++it)
    {
        uint32_t sectionStartAddress;
        uint32_t sectionEndAddress;
        uint32_t sectionSize;
        const section *pSection = *it;

        sectionStartAddress = (uint32_t)pSection->get_address();
        sectionSize = (uint32_t)pSection->get_size();
        sectionEndAddress = sectionStartAddress + sectionSize;

        if(SHT_PROGBITS == pSection->get_type() &&
            0 != (SHF_ALLOC & pSection->get_flags()))
        {
            //
            // At least 1 byte from this section is within the memory region.
            //

            // Remove the offset from the start address to use it as an index
            // into the memory region buffer.
            sectionStartAddress -= startAddress;

#ifdef DEBUG_PRINT
            printf("Copying section %s: 0x%08x-0x%08x (%u bytes) to %u\n",
                pSection->get_name().c_str(),
                sectionStartAddress + startAddress, sectionEndAddress, sectionSize, sectionStartAddress);
#endif // DEBUG_PRINT

            // Read the section data.
            memcpy(&pMemory[sectionStartAddress], pSection->get_data(),
                sectionSize);
        }
    }

#ifdef DEBUG_PRINT
    printf("All sections copied.\n");
#endif
    return pMemory;
}


uint64_t get_section_size(void* elf, const char* section_name)
{
    if(!elf) return 0;
    elfio2* pElf = (elfio2*)elf;

    // get the section with a given name.
    section *pSec = pElf->sections[section_name];
    if(!pSec) return 0;

    return pSec->get_size();
}

uint64_t get_section_addr(void* elf, const char* section_name)
{
    if(!elf) return INVALID_ADDRESS;
    elfio2* pElf = (elfio2*)elf;

    // get the section with a given name.
    section *pSec = pElf->sections[section_name];
    if(!pSec) return INVALID_ADDRESS;

    return pSec->get_address();

}

uint64_t get_symbol_addr(void* elf, const char* symbol_name)
{
    if(!elf) return INVALID_ADDRESS;
    elfio2* pElf = (elfio2*)elf;

    // get the section with a given name.
    symbol *pSym = pElf->symbols[symbol_name];
    if(!pSym) return INVALID_ADDRESS;

    return pSym->get_value();

}

const char* get_section_data(void* elf, const char* section_name)
{
    if(!elf) return NULL;
    elfio2* pElf = (elfio2*)elf;

    // get the section with a given name.
    section *pSec = pElf->sections[section_name];
    if(!pSec) return NULL;


    const char* data = pSec->get_data();
    if(!data)
    {
        // check in initdat
        const char* data = get_section_data(elf, STR_INITDAT);
        uint64_t size = get_section_size(elf, STR_INITDAT);
        InitData* initdata = new InitData();
        initdata->Load((const uint8_t*)data, size);

        return (const char*)initdata->GetDataAtAddress(pSec->get_address(), pSec->get_size());
    }
    else
    {
        return data;
    }

}

bool set_section_data(void* elf, const char* section_name, const char* data, uint64_t size)
{
    Elf_Word wsize = size;
    elfio2* pElf = (elfio2*)elf;
    if(!pElf)
    {
        return false;
    }

    // get the section with a given name.
    section *pSec = pElf->sections[section_name];
    if(!pSec)
    {
        return false;
    }

    if(pSec->get_size() != size)
    {
        return false;
    }

    if(pSec->get_data())
    {
        // Data is located in an allocated area.
        pSec->set_data((const char*)data, wsize);
        return true;
    }
    else
    {
        // Attempt to update the section in initdata.
        section *pInitSec = pElf->sections[STR_INITDAT];
        if(!pInitSec)
        {
            return false;
        }

        const char* init = pInitSec->get_data();
        uint64_t initsize = pInitSec->get_size();
        InitData initdata;
        initdata.Load((const uint8_t*)init, initsize);

        if(initdata.SetDataAtAddress(pSec->get_address(), size, (const uint8_t*)data))
        {
            uint32_t new_size;
            uint8_t* new_data = initdata.Generate(new_size);

            pInitSec->set_data((const char*)new_data, new_size);

            delete[] new_data;
            return true;
        }

        return false;
    }
}
