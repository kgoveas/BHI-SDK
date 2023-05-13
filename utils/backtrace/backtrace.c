////////////////////////////////////////////////////////////////////////////////
///
/// @file       backtrace.c
///
/// @project    7189
///
/// @brief      7189 Backtrace utility. Based on the simplereader.c libdwarf
///                 example.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016 EM Microelectronic
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
/*
  Copyright (c) 2009-2010 David Anderson.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of the example nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY David Anderson ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL David Anderson BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <sys/types.h> /* For open() */
#include <sys/stat.h>  /* For open() */
#include <fcntl.h>     /* For open() */
#include <stdlib.h>     /* For exit() */
#include <unistd.h>     /* For close() */
#include <string.h>     /* For strcmp* */
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>

#include <err.h>
#include <fcntl.h>
#include <gelf.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sysexits.h>
#include <unistd.h>

#include <libelf.h>
#include <gelf.h>

#include "dwarf.h"
#include "libdwarf.h"

#define MIN_INSTRUCTION_SIZE    (2)

typedef unsigned char  UInt8;
typedef unsigned short UInt16;
typedef unsigned int   UInt32;

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
        };
    };
    UInt32 pc;
} em_context_t;

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
    UInt8  reserved[16];    // 0xb4
    UInt32 stackCrc;        // 0xc4
    UInt32 crc;             // 0xc8
    // UInt8 stack[0]; // Stack is copied to the end of this structure
} post_mortem_info;

typedef struct {
    const char* filename;
    /* Elf Entries */
    Elf*    elf;
    int     fd;

    /* Dwarf Entries */
    bool dbg_loaded;
    Dwarf_Debug dbg;
} binary_t;


UInt32 update_crc_32(UInt32 crc_accum, UInt32* data_blk_ptr, UInt32 data_blk_size);

static void print_backtrace(binary_t binaries[], int num_bin, post_mortem_info* pm_info);
static void print_registers(post_mortem_info *pmi, binary_t binaries[], int num_bin);
static UInt32 get_cfa  (Dwarf_Fde fde, em_context_t* context);
static int unwind_frame(Dwarf_Fde fde, post_mortem_info* pm_info);
static void dump_stack(UInt8* stackbuf, UInt32 stackAddr, UInt16 stackSize);

UInt8 stack_included = 0;
UInt32 stack_bottom = 0;
UInt32 stack_top = 0;
UInt8* stack;
#define STACK_BOTTOM     stack_bottom
#define STACK_SIZE       (stack_top - stack_bottom)

#define UNDEF_VAL 2000
#define SAME_VAL  2001
#define CFA_VAL   2002

#define PM_FLAG_HOST_INT_CTRL_VALID     0x01
#define PM_FLAG_STACK_INFO_VALID        0x02
#define PM_FLAG_STACK_CONTENTS_INCLUDED 0x04

int usage(int argc, char *argv[])
{
    if(argc < 2)
    {
        fprintf(stderr, "Please specify a post mortem debug dump file.\n");
    }

    fprintf(stderr, "%s pm.bin [rom.elf] [kernel.elf] [user.elf]\n", argv[0]);


    return 1;
}

Elf* load_elf(const char* path, int *fd)
{
    int mfd = -1;
    Elf* e;


    if (elf_version(EV_CURRENT) == EV_NONE )
    {
        errx(EX_SOFTWARE, "ELF library initialization failed : %s ", elf_errmsg(-1));
    }

    if ((mfd = open(path, O_RDONLY , 0)) < 0)
    {
        err(EX_NOINPUT,"open %s failed ", path);
    }

    if ((e = elf_begin(mfd, ELF_C_READ , NULL)) == NULL)
    {
        errx(EX_SOFTWARE, "elf_begin () failed : %s . ", elf_errmsg(-1));
    }

    if (elf_kind(e) != ELF_K_ELF)
    {
        errx(EX_DATAERR, "%s is not an ELF object . ", path);
    }

    if(fd) *fd = mfd;
    return e;
}

int close_dwarf(Dwarf_Debug* dbg)
{
    Dwarf_Error error;

    int res = dwarf_finish(*dbg, &error);

    if(res != DW_DLV_OK) {
        printf("dwarf_finish failed!\n");
    }

    return res;
}

int close_elf(int fd, Elf* e)
{
    int res = close(fd);
    res |= elf_end(e);

    return res;
}

int load_dwarf(int fd,
    Dwarf_Unsigned access,
    Dwarf_Handler  errhand,
    Dwarf_Ptr      errarg,
    Dwarf_Debug*   dbg,
    Dwarf_Error*   error)
{
    int regtabrulecount = 0;
    int res;

    res = dwarf_init(fd,DW_DLC_READ,errhand,errarg, dbg, error);
    if(res != DW_DLV_OK) {
        return -1;
    }
    /*  Do this setting after init before any real operations.
        These return the old values, but here we do not
        need to know the old values.  The sizes and
        values here are higher than most ABIs and entirely
        arbitrary.

        The setting of initial_value to
        the same as undefined-value (the other possible choice being
        same-value) is arbitrary, different ABIs do differ, and
        you have to know which is right. */
    regtabrulecount=1999;
    dwarf_set_frame_undefined_value(*dbg, UNDEF_VAL);
    dwarf_set_frame_rule_initial_value(*dbg, UNDEF_VAL);
    dwarf_set_frame_same_value(*dbg,SAME_VAL);
    dwarf_set_frame_cfa_value(*dbg,CFA_VAL);
    dwarf_set_frame_rule_table_size(*dbg,regtabrulecount);


    return res;
}

int main(int argc, char *argv[])
{
    int fddebug;
    int ret = 0;
    int res = DW_DLV_ERROR;
    Dwarf_Error error;
    Dwarf_Handler errhand = 0;
    Dwarf_Ptr errarg = 0;
    char* pmFile = NULL;

    int i;
    int max_binaries = argc - 2;
    binary_t *binaries = calloc(max_binaries, sizeof(binary_t));
    for (i = 0; i < max_binaries; i++)
    {
        // Invalidate all file descriptors
        binaries[i].fd = -1;
    }

    int context_size;
    struct stat stats;
    post_mortem_info pm_info;
    UInt8* stackbuf;
    UInt32 crc;

    if(argc < 2)
    {
        exit(usage(argc, argv));
    }

    pmFile = argv[1];
    for(i = 2; i < argc; i++)
    {
        binaries[i-2].filename = argv[i];
    }

    // Determine size of debug dump file
    if ((fddebug =open(pmFile, O_RDONLY , 0)) < 0)
    {
        err(EX_NOINPUT,"open %s failed ", pmFile);
    }
    if(fstat(fddebug, &stats))
    {
        err(EX_NOINPUT,"stat %s failed ", pmFile);
    }
    context_size = stats.st_size;

    // read in context
    if(context_size >= sizeof(post_mortem_info))
    {
        read(fddebug, &pm_info, sizeof(pm_info));
        if (pm_info.flags & PM_FLAG_STACK_CONTENTS_INCLUDED)
        {
            stack_included = 1;
            stack_bottom = pm_info.em_ctx.sp;
            stack_top = stack_bottom + (context_size - sizeof(pm_info));
            stackbuf = (UInt8*)malloc(STACK_SIZE);
            read(fddebug, stackbuf, STACK_SIZE);
            stack = stackbuf + stack_bottom - pm_info.stackStart;

            crc = 0xFFFFFFFFU;
            crc = update_crc_32(crc, (UInt32*)stackbuf, pm_info.stackSize/4);
            if (crc != pm_info.stackCrc)
            {
                printf("CRC error on post mortem stack: calculated CRC: %x, saved CRC %x\n", crc, pm_info.stackCrc);
            }
        }

        ret |= close(fddebug);

        crc = 0xFFFFFFFFU;
        crc = update_crc_32(crc, (UInt32*)&pm_info, sizeof(pm_info)/4 - 1);
        if (crc != pm_info.crc)
        {
            printf("CRC error on post mortem info: calculated CRC: %x, saved CRC %x\n", crc, pm_info.crc);
        }
    }

    for(i = 0; i < max_binaries; i++)
    {
        if(binaries[i].filename)
        {
            const char *filename = binaries[i].filename;
            printf("Loading elf file %s\n", filename);
            binaries[i].elf = load_elf(filename, &binaries[i].fd);
            if(binaries[i].fd >= 0)
            {
                int dwarf_res = load_dwarf(binaries[i].fd, DW_DLC_READ, errhand, errarg, &binaries[i].dbg, &error);
                if(dwarf_res >= 0)
                {
                    binaries[i].dbg_loaded = true;
                }
            }
        }
    }

    if (stack_included)
    {
        dump_stack(stackbuf, pm_info.stackStart, pm_info.stackSize);
    }

    print_registers(&pm_info, binaries, max_binaries);


    print_backtrace(binaries, max_binaries, &pm_info);

    for(i = 0; i < max_binaries; i++)
    {
        if(binaries[i].dbg_loaded)
        {
            res |= close_dwarf(&binaries[i].dbg);
        }

        if(binaries[i].fd >= 0)
        {
            res |= close_elf(binaries[i].fd, binaries[i].elf);
        }
    }

    free(binaries);

    return ret;
}


static char*
function_for_address(long address, binary_t binaries[], int num_bin, int *offset)
{
    int elfidx;
    for(elfidx = 0; elfidx < num_bin; elfidx++)
    {
        Elf* e = binaries[elfidx].elf;
        if(NULL == e)
        {
            continue;
        }

        Elf_Scn *scn=NULL;
        GElf_Shdr shdr;
        int symbols = 0;
        int i;

        Elf_Data  *edata = NULL;
        GElf_Sym   symbol;

        while ((scn = elf_nextscn(e, scn)) != NULL) {
                gelf_getshdr(scn, &shdr);

            if(shdr.sh_type == SHT_SYMTAB)
            {
                edata = elf_getdata(scn, edata);

                // found symbol table, look for our address
                symbols = shdr.sh_size / shdr.sh_entsize;

                for(i = 0; i < symbols; i++)
                {
                    gelf_getsym(edata, i, &symbol);

                    if(ELF32_ST_TYPE(symbol.st_info) == STT_FUNC)
                    {
                        if((symbol.st_value <= (long)address) &&
                           ((symbol.st_value + symbol.st_size) > (long)address))
                        {
                            //printf("Found symbol '%s' for address %p\n", elf_strptr(e, shdr.sh_link, symbol.st_name), address);
                            *offset = address - symbol.st_value;
                            return elf_strptr(e, shdr.sh_link, symbol.st_name);
                        }
                    }
                }
            }
        }
    }

    *offset = address;
    return NULL;
}


static UInt32 get_cfa(Dwarf_Fde fde, em_context_t* context)
{
    Dwarf_Error error;
    Dwarf_Signed reg = 0;
    Dwarf_Signed offset_relevant = 0;
    Dwarf_Small value_type = 0;
    Dwarf_Signed offset_or_block_len = 0;
    Dwarf_Ptr block_ptr = 0;
    Dwarf_Addr row_pc = 0;
    Dwarf_Addr  pc = context->pc;
    int res;
    UInt32 cfa = 0;



    // (1) Locate the CFA register (should be r28 on the arc processor)
    res = dwarf_get_fde_info_for_cfa_reg3(fde,
        pc,
        &value_type,
        &offset_relevant,
        &reg,
        &offset_or_block_len,
        &block_ptr,
        &row_pc,
        &error);

    if(res != DW_DLV_OK) {
        if(DW_DLE_CFE_INFO_UNKNOWN != dwarf_errno(error))
        {
            printf("dwarf_get_fde_info_for_cfa_reg PC %llx, res is %llu\n", pc, dwarf_errno(error));
            exit(1);
        }
        else
        {
            return 0;
        }
    }

    switch (value_type)
    {
        case DW_EXPR_OFFSET:
        case DW_EXPR_VAL_OFFSET:
            if(offset_relevant)
            {
                cfa = context->reg[reg] + offset_or_block_len;
            }
            else
            {
                cfa = context->reg[reg];
            }
            break;

        case DW_EXPR_EXPRESSION:
        case DW_EXPR_VAL_EXPRESSION:
            cfa = context->reg[reg];
            break;
    }

    return cfa;
}
static bool parse_interrupt(post_mortem_info* pm_info)
{
    if(pm_info->ecr || (pm_info->em_ctx.sp == 0))
    {
        if(pm_info->eret == pm_info->em_ctx.pc)
        {
            // Infinite loop. Exit.
            return false;
        }
        // This was an exception
        printf(" (Exception)\n");
        pm_info->em_ctx.pc = pm_info->eret;
    }
    else
    {
        if(pm_info->em_ctx.ilink == pm_info->em_ctx.pc)
        {
            // Infinate loop. Exit.
            return false;
        }
        // return from interrupt
        printf(" (Interrupt)\n");
        pm_info->em_ctx.pc = pm_info->em_ctx.ilink;
    }

    return true;
}
static int unwind_frame(Dwarf_Fde fde, post_mortem_info* pm_info)
{
    Dwarf_Error error;
    Dwarf_Signed reg = 0;
    Dwarf_Signed offset_relevant = 0;
    Dwarf_Small value_type = 0;
    Dwarf_Signed offset_or_block_len = 0;
    Dwarf_Ptr block_ptr = 0;
    Dwarf_Addr row_pc = 0;
    Dwarf_Addr  pc = pm_info->em_ctx.pc;
    int res;
    int i;
    int is_int = 0;
    UInt32 cfa = get_cfa(fde, &pm_info->em_ctx);

    //printf(", CFA: 0x%.8x (%d) for pc 0x%.8x, ", cfa, cfa - STACK_BOTTOM, pm_info->em_ctx.pc);

    //printf("\n");
    for(i = 0; i < sizeof(pm_info->em_ctx.reg)/sizeof(pm_info->em_ctx.reg[0]); i++)
    {
        // (2) Locate register
        res = dwarf_get_fde_info_for_reg3(fde,i, // reg
            pc,
            &value_type,
            &offset_relevant,
            &reg,
            &offset_or_block_len,
            &block_ptr,
            &row_pc,
            &error);

        if(res != DW_DLV_OK)
        {
            // Error.
            printf("\n");
            return 0;
        }

        switch(reg)
        {
            default:
                pm_info->em_ctx.reg[i] = -1;
                is_int = 1;
                break;

            case UNDEF_VAL: // unknown register value for frame. Ignore
                // Hack to allow blink to unwind.
                if(i != 31) pm_info->em_ctx.reg[i] = -1;
            case SAME_VAL:  // register has not changed.
                break;

            case CFA_VAL:   // CFA relative
                switch (value_type)
                {
                    case DW_EXPR_OFFSET:
                    case DW_EXPR_VAL_OFFSET:
                        if(offset_relevant)
                        {
                            int index = ((cfa - STACK_BOTTOM) + offset_or_block_len);
                            if(index > STACK_SIZE)
                            {
                                printf("\n\tERROR: Unable to locate stack data at index %d\n", index);
                                return 0;
                            }

                            index /= 4;
                            if(i == 31)
                            {
                                //printf("Register r%d offset is %d\n", i, offset_or_block_len);
                                //printf("  Looking at %x, %x\n", index, cfa + index);
                            }
                            UInt32* stack32 = (void*)stack;
                            //if(i == 31) printf("  value is %x\n", stack32[index]);
                            pm_info->em_ctx.reg[i] = stack32[index];
                        }
                        else
                        {
                            //?
                            //printf("r%d is r%d\n", i, reg);
                        }
                        break;

                    case DW_EXPR_EXPRESSION:
                    case DW_EXPR_VAL_EXPRESSION:
                        printf("Case not handled\n");
                        break;
                }
                break;
        }
    }

    if((pm_info->em_ctx.pc == pm_info->em_ctx.blink - MIN_INSTRUCTION_SIZE) &&
        ((pm_info->em_ctx.sp == cfa) || (pm_info->em_ctx.sp == -1)))
    {
        // end of backtrace - infinite loop.
        printf("\n");
        return 0;
    }

    // registers unwound. Retore stack pointer and return from function
    if(is_int)
    {
        if(parse_interrupt(pm_info))
        {
            pm_info->em_ctx.sp = cfa;
        }
        else
        {
            // Infinite loop. Exit.
            printf("\n");
            return 0;
        }
    }
    else
    {
        printf("\n");
        pm_info->em_ctx.sp = cfa;
        if(pm_info->em_ctx.blink)
        {
            // Go to the previous instruction. Instructions are at least 2 bytes.
            pm_info->em_ctx.pc = pm_info->em_ctx.blink - MIN_INSTRUCTION_SIZE;
        }
        else
        {
            return 0;
        }
    }

    if (pm_info->em_ctx.pc == 0)
    {
        return 0;
    }
    return 1;
}


int get_fde_for_pc(binary_t binaries[], int num_bin, Dwarf_Addr pc, Dwarf_Fde* fde, Dwarf_Addr* lopc, Dwarf_Addr* hipc, Dwarf_Error* error)
{
    int i;
    int res = DW_DLV_ERROR;
    Dwarf_Signed cie_element_count = 0;
    Dwarf_Signed fde_element_count = 0;
    Dwarf_Cie *cie_data = 0;
    Dwarf_Fde *fde_data = 0;


    for(i = 0; i < num_bin; i++)
    {
        if(binaries[i].dbg_loaded)
        {
            Dwarf_Debug dbg = binaries[i].dbg;

            //printf("\n\tSearching dbg %p for pc %p\n", dbgs, pc);

            res = dwarf_get_fde_list(dbg, &cie_data, &cie_element_count,
                &fde_data, &fde_element_count,error);

            if(res == DW_DLV_OK)
            {
                // got fde list, look for our fde
                res = dwarf_get_fde_at_pc(fde_data, pc, fde, lopc, hipc, error);

                if(res == DW_DLV_OK)
                {
                    // FOUND, will be exiting soon.. leak the memory...
                }
                else
                {
                    // TODO: free fde list
                }
            }
            else
            {
                // TODO: free fde list?
            }
        }

        if(DW_DLV_OK == res)
        {
            // Debug info found.
            break;
        }
    }

    return res;
}

static void print_context(binary_t binaries[], int num_bin, post_mortem_info* pm_info)
{
    int i;
    int offset;
    uint32_t pc = pm_info->em_ctx.pc;
    char* name = function_for_address(pc, binaries, num_bin, &offset);

    // Printout function address and name
    printf("0x%.8X: %s", pc, name ? name : "(null)");
    if(offset)
    {
        printf("+%d", offset);
    }

    printf("\n\t");
    for(i = 0; i < 28; i++)
    {
        printf("<r%d>=0x%.8x, ", i, pm_info->em_ctx.reg[i]);
        if (((i+1)%4) == 0)
            printf("\n\t");
    }
    printf("<sp>=0x%.8x, <ilink>=0x%.8x, <r30>=0x%.8x, <blink>=0x%.8x\n",
        pm_info->em_ctx.reg[28], pm_info->em_ctx.reg[29], pm_info->em_ctx.reg[30], pm_info->em_ctx.reg[31]);
}

static void print_backtrace(binary_t binaries[], int num_bin, post_mortem_info* pm_info)
{
    Dwarf_Error error;
    int res;

    Dwarf_Fde   myfde;

    do
    {
        Dwarf_Addr  lopc = 0;
        Dwarf_Addr  hipc = 0;

        print_context(binaries, num_bin, pm_info);

        res = get_fde_for_pc(binaries, num_bin, pm_info->em_ctx.pc, &myfde, &lopc, &hipc, &error);
        if(res != DW_DLV_OK)
        {
            int offset;
            char* name = function_for_address(pm_info->em_ctx.pc, binaries, num_bin, &offset);

            lopc = 0;
            hipc = 0;

            if(name && 0 == strcmp(name, "NullHandler"))
            {
                // Special case - Allow NullHandler to be parsed even without debug info.
                if(parse_interrupt(pm_info))
                {
                    print_context(binaries, num_bin, pm_info);

                    res = get_fde_for_pc(binaries, num_bin, pm_info->em_ctx.pc, &myfde, &lopc, &hipc, &error);
                }
            }
            else if(name && 0 == strcmp(name, "PM_fatalError"))
            {
                printf("\n");
                // Special case - Allow PM_fatalError to be parsed even without debug info.
                pm_info->em_ctx.pc = pm_info->em_ctx.reg[31] - MIN_INSTRUCTION_SIZE;
                pm_info->em_ctx.sp += 8; // 2 registers pushed on the stack.

                print_context(binaries, num_bin, pm_info);

                res = get_fde_for_pc(binaries, num_bin, pm_info->em_ctx.pc, &myfde, &lopc, &hipc, &error);
            }


            if(res != DW_DLV_OK)
            {
                char* name = function_for_address(pm_info->em_ctx.pc, binaries, num_bin, &offset);
                printf("Debug information not present for %s (at %x), ending backtrace early.\n", name, pm_info->em_ctx.pc);
                break;
            }
        }
    } while(unwind_frame(myfde, pm_info));

}

static void print_register(const char* name, uint32_t value, binary_t binaries[], int num_bin, bool previous_addr)
{
    int offset = 0;
    bool printOffset = true;
    const char* symbol;

    if(previous_addr)
    {
        // Print the symbol name based on the previous address.
        // This is used in cases like blink where we want to know the calling function.
        // In some cases, functions marked as noreturn would result in the next function
        // be printed instead, causing a confusing printout. This fixes those cases.
        symbol = function_for_address(value - MIN_INSTRUCTION_SIZE, binaries, num_bin, &offset);
        offset += MIN_INSTRUCTION_SIZE;
    }
    else
    {
        symbol = function_for_address(value, binaries, num_bin, &offset);
    }

    if(!symbol)
    {
        printOffset = false;
        symbol = "";
    }

    printf("%9s 0x%.8x %s", name, value, symbol);
    if(offset && printOffset)
    {
        printf("+%d\n", offset);
    }
    else
    {
        printf("\n");
    }
}

static void print_registers(post_mortem_info *pmi, binary_t binaries[], int num_bin)
{
    int i;
    for (i=0; i<26; i++)
    {
        char regname[] = "rXX";
        sprintf(regname, "r%d", i);
        print_register(regname,   pmi->em_ctx.reg[i], binaries, num_bin, false);
    }

    print_register("gp r26",    pmi->em_ctx.reg[26], NULL, 0, false);
    print_register("fp r27",    pmi->em_ctx.reg[27], NULL, 0, false);
    print_register("sp r28",    pmi->em_ctx.reg[28], NULL, 0, false);
    print_register("ilink r29", pmi->em_ctx.reg[29], binaries, num_bin, false);
    print_register("r30",       pmi->em_ctx.reg[30], binaries, num_bin, false);
    print_register("blink r31", pmi->em_ctx.reg[31], binaries, num_bin, true);
    print_register("pc",        pmi->em_ctx.pc, binaries, num_bin, false);
    print_register("eret",      pmi->eret, binaries, num_bin, false);
    print_register("erbta",     pmi->erbta,binaries, num_bin, false);
    print_register("erstatus",  pmi->erstatus, NULL, 0, false);
    print_register("ecr",       pmi->ecr, NULL, 0, false);
    print_register("efa",       pmi->efa, binaries, num_bin, false);
    print_register("icause",    pmi->icause, NULL, 0, false);
    print_register("mpu_ecr",   pmi->mpu_ecr, NULL, 0, false);
    printf("\n");

    printf("       diag 0x%.8x\n", pmi->diagnostic);
    printf("debug state 0x%.8x\n", pmi->debugState);
    printf("  debug val 0x%.8x\n", pmi->debugValue);
    printf("  error val 0x%.8x\n", pmi->errorReason);
    printf("  interrupt 0x%.8x\n", pmi->interruptState);
    printf(" err report 0x%.8x\n", pmi->errorReport);
    printf("\n");

    printf("  stack start 0x%.8x\n", pmi->stackStart);
    printf("stack pointer 0x%.8x\n", pmi->em_ctx.reg[28]);
    printf("   stack size 0x%.8x\n", pmi->stackSize);
    printf(" reset reason 0x%.8x\n", pmi->resetReason);
    printf("\n");

    printf("    stack CRC 0x%.8x\n", pmi->stackCrc);
    printf("          CRC 0x%.8x\n", pmi->crc);
    printf("\n");
}

static void dump_stack(UInt8* stackbuf, UInt32 stackAddr, UInt16 stackSize)
{
    UInt32 addr = stackAddr;
    UInt16 size;
    UInt32* bufPtr = (UInt32*)stackbuf;

    printf("Stack contents:\n");
    for (size = 0; size < stackSize; size+=16)
    {
        printf("0x%.8x: ", addr);
        printf("0x%.8x 0x%.8x 0x%.8x 0x%.8x\n", bufPtr[0], bufPtr[1], bufPtr[2], bufPtr[3]);
        addr += 16;
        bufPtr += 4;
    }
    printf("\n\n");
}
