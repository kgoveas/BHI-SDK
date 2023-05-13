////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/initdat/initdatgen.cpp
///
/// @project    EM7189
///
/// @brief      Tool to generate the information from the .initdat elf section.
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

#include <cstdio>
#include <cstdlib>

#include <list>
#include <string>

#include <InitData.h>

#include <elfio/elfio2.hpp>

#if WIN32
#include <Windows.h>
#include <io.h>

#define err(code, ...) do { \
    fprintf (stderr, __VA_ARGS__); \
    fprintf (stderr, "\n"); \
    exit (code); \
} while (0)

#define errx(code, ...) do { \
    fprintf (stderr, __VA_ARGS__); \
    fprintf (stderr, "\n"); \
    exit (code); \
} while (0)
#else
#include <err.h>
#endif // WIN32

using namespace ELFIO;

bool gQuiet = false;
#define verbose(...) do { \
    if(!gQuiet) \
    { \
        printf (__VA_ARGS__); \
        printf ("\n"); \
    } \
} while (0)


bool CheckRule(const std::list<std::string>& rules, const section *pSection,
    bool& isBSS)
{
    if( (0 == (pSection->get_flags() & SHF_ALLOC)) ||   /* Not allocated */
        (0 == pSection->get_size()) ||                  /* No Size */
        (0 != (pSection->get_flags() & SHF_EXECINSTR))  /* Text / code */
        )
    {
        return false;
    }

    const char *pData = pSection->get_data();

    bool zero = true;
    for(Elf_Xword i = 0; 0 != pData && zero &&
        i < pSection->get_size(); ++i)
    {
        if(0 != pData[i])
        {
            zero = false;
        }
    }

    isBSS = zero;
    bool haveMatch = false;

    for(std::list<std::string>::const_iterator it = rules.begin();
        it != rules.end(); ++it)
    {
        const std::string& rule = *it;

        if(rule.empty())
        {
            continue;
        }

        switch(rule[0])
        {
            case '!':
            {
                if("!bss" == rule)
                {
                    if(isBSS)
                    {
                        haveMatch = true;
                    }
                }
                else if("!data" == rule)
                {
                    if(0 != (pSection->get_flags() & SHF_WRITE))
                    {
                        if(!isBSS)
                        {
                            haveMatch = true;
                        }
                    }
                }
                else if("!lit" == rule)
                {
                    if(0 == (pSection->get_flags() & SHF_WRITE))
                    {
                        if(!isBSS)
                        {
                            haveMatch = true;
                        }
                    }
                }
                else
                {
                    fprintf(stderr, "Bad rule: %s\n", rule.c_str());
                }
                break;
            }
            case '~':
            {
                if(rule.substr(1) == pSection->get_name())
                {
                    return false;
                }
                break;
            }
            default:
            {
                if(rule == pSection->get_name())
                {
                    return true;
                }
                break;
            }
        }
    }

    return haveMatch;
}

int main(int argc, char *argv[])
{
    int argumentOffset = 0;
    bool nocompress = false;

    if(1 < argc && std::string("-q") == argv[argumentOffset + 1])
    {
        gQuiet = true;
        argumentOffset++;
    }

    if(1 < argc && std::string("-Xnocompress") == argv[argumentOffset + 1])
    {
        nocompress = true;
        argumentOffset++;
    }

    if( (nocompress && 5 > argc) || (!nocompress && 4 > argc))
    {
        fprintf(stderr, "USAGE: %s [-q] [-Xnocompress] IN OUT SECTION..\n", argv[0]);

        return -1;
    }

    elfio2 elf;

    if(!elf.load(argv[argumentOffset + 1]))
    {
        errx(EXIT_FAILURE, "Failed to read ELF file.");
    }

    const symbol *pSymbol = elf.symbols["_initdat"];

    if(0 == pSymbol)
    {
        errx(EXIT_FAILURE, "ELF file does not have a _initdat symbol.");
    }

    if(0 != elf.sections[".initdat"])
    {
        errx(EXIT_FAILURE, "ELF file has a .initdat section already.");
    }

    const char *szOut = argv[argumentOffset + 2];

    std::list<std::string> rules;

    for(int i = argumentOffset + 3; i < argc; ++i)
    {
        rules.push_back(argv[i]);
    }

    InitData initdat;

    const symbol *pDecompress = elf.symbols["_lz_decompress"];

    if(0 != pDecompress && !nocompress)
    {
        initdat.SetCompressed(true);
        initdat.SetDecompressAddress((uint32_t)pDecompress->get_value());
    }

    for(std::vector<section*>::const_iterator it = elf.sections.begin();
        it != elf.sections.end(); ++it)
    {
        section *pSection = *it;

        bool isBSS;

        if(CheckRule(rules, pSection, isBSS) && !isBSS)
        {
            verbose("Adding Section: %s", pSection->get_name().c_str());
            initdat.AddEntry(new InitDataEntry(
                (uint32_t)pSection->get_address(),
                (uint8_t*)pSection->get_data(),
                (uint32_t)pSection->get_size()));

            pSection->set_type(SHT_NOBITS); /* Remove the section from the elf file, now in initdata. */
        }
    }

    for(std::vector<section*>::const_iterator it = elf.sections.begin();
        it != elf.sections.end(); ++it)
    {
        section *pSection = *it;

        bool isBSS;

        if(CheckRule(rules, pSection, isBSS) && isBSS)
        {
            verbose("Adding Section: %s", pSection->get_name().c_str());

            initdat.AddEntry(new InitDataEntry(
                (uint32_t)pSection->get_address(),
                (uint32_t)pSection->get_size()));

            pSection->set_type(SHT_NOBITS); /* Remove the section from the elf file, now in initdata. */
        }
    }

    uint32_t size = 0;
    uint8_t *pData = initdat.Generate(size);

    section *pInitdat = elf.sections.add(".initdat");
    pInitdat->set_type(SHT_PROGBITS);
    pInitdat->set_flags(SHF_ALLOC);
    pInitdat->set_addr_align(4);
    pInitdat->set_address(pSymbol->get_value());
    pInitdat->set_data((char*)pData, size);

    segment *pSegment = elf.segments.add();
    pSegment->set_type(PT_LOAD);
    pSegment->set_virtual_address(pInitdat->get_address());
    pSegment->set_physical_address(pInitdat->get_address());
    pSegment->set_flags(PF_R);
    pSegment->set_align(pInitdat->get_addr_align());
    pSegment->add_section_index(pInitdat->get_index(),
        pInitdat->get_addr_align());
    pSegment->set_file_size(size);
    pSegment->set_memory_size(size);

    if(!elf.save(szOut))
    {
        errx(EXIT_FAILURE, "Failed to write output ELF file: %s", szOut);
    }

    delete[] pData;

    return EXIT_SUCCESS;
}
