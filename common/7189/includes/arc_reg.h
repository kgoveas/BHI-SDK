////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/arc_reg.h
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
#ifndef _ARC_REG_H_
#define _ARC_REG_H_
//lint -save -e760
/*****************************************************************************
   ARC International
   Register Definition File
   Customer Support
   NOV 2009


    Core register defines are useful with C compiler intrinsics _core_read,
    _core_write as in:

       int low_part = _core_read(MLO_REG);

    Aux register defines are useful with C compiler intrinsics _lr(reg)
    and _sr(value,reg);

    7.4.3 compilers with patch #1 or later tools allow use of -Hasmcpp and
    $reg(N) to convert a defined number to a register name.
    I.e. $reg(26) converts to %gp, for example. This effectively means
    that the same header file can be used on assembler with #include as on C.

    Useful timer macros:

      START_TIMER0(), GET_TIMER0(), START_TIMER1(), GET_TIMER1()

    NOTE: register names are derived from the system reference guide.
    ARC700 4.2 release.   The register names  have REG_ prepended and
    are converted to upper case in the C convention for
    #define REG_constants.  This makes user define conflicts less
    likely but does not guarantee it cannot happen.  Bit and mask
    values are supplied where possible.

    For Aux registers, symbols are also defined without the REG_
    for convenience. If these are not wanted or if name conflicts
    occur in your project, prior to including this file,
    define the symbol "_AUX_DEFINES_NOT_WANTED".

    NOTE: this document is not official ARC International source and
    is provided solely as a convenience to customers by Customer Support.
    Register names may change in the future as feedback is given from
    users and engineering.


    Revision history at file end.


******************************************************************/

#ifdef __Xxy
#include "xy.h"
#ifdef __CCAC__
    #include "xyregs.h"
#endif
#endif /* __Xxy */


// Core register allocation.

#define REG_R0  (0)  //  Integer result; argument 1 Volatile scratch register
#define REG_R1  (1)  //  Argument 2 Volatile scratch register
#define REG_R2  (2)  //  Argument 3 Volatile scratch register
#define REG_R3  (3)  //  Argument 4 Volatile scratch register
#define REG_R4  (4)  //  Argument 5 Volatile scratch register
#define REG_R5  (5)  //  Argument 6 Volatile scratch register
#define REG_R6  (6)  //  Argument 7 Volatile scratch register
#define REG_R7  (7)  // Argument 8 Volatile scratch register
#define REG_R8  (8)  // Volatile scratch register .
#define REG_R9  (9)  // Volatile scratch register .
#define REG_R10 (10) // Volatile scratch register .
#define REG_R11 (11) // Volatile scratch register Used to pass the static link for supporting nested functions
#define REG_R12 (12) // Volatile scratch register used for temporary calculations
#define REG_R13 (13) // Non-volatile register variable (callee saved)
#define REG_R14 (14) // Non-volatile register variable (callee saved)
#define REG_R15 (15) // Non-volatile register variable (callee saved)
#define REG_R16 (16) // Non-volatile register variable (callee saved)
#define REG_R17 (17) // Non-volatile register variable (callee saved)
#define REG_R18 (18) // Non-volatile register variable (callee saved)
#define REG_R19 (19) // Non-volatile register variable (callee saved)
#define REG_R20 (20) // Non-volatile register variable (callee saved)
#define REG_R21 (21) // Non-volatile register variable (callee saved)
#define REG_R22 (22) // Non-volatile register variable (callee saved)
#define REG_R23 (23) // Non-volatile register variable (callee saved)
#define REG_R24 (24) // Non-volatile register variable (callee saved)
#define REG_R25 (25) // Non-volatile register variable (callee saved)
#define REG_GP (26) // Small-data base register (gp)
#define REG_R26 (26) // Small-data base register (gp)
#define REG_FP (27) // Frame pointer (fp)
#define REG_R27 (27) // Frame pointer (fp)
#define REG_SP (28) // Stack top pointer (sp)
#define REG_R28 (28) // Stack top pointer (sp)
#define REG_ILINK1 (29) // Maskable interrupt link register 1 (ilink1)
#define REG_ILINK (29) // ARCv2 cores just have "ilink"
#define REG_R29 (29) // Maskable interrupt link register 1 (ilink1)
#define REG_ILINK2 (30) // Maskable interrupt link register 2 (ilink2)
#define REG_R30 (30) // Maskable interrupt link register 2 (ilink2)
#define REG_BLINK (31) //  Branch link register (blink) .
#define REG_R31 (31) //  Branch link register (blink) .
#define REG_X0_U0 (32) //  r/w  32  XY MEMORY DSP v3.1  Pointer x0 with update      mode 0
#define REG_X0_U1 (33) //  r/w  32  XY MEMORY DSP v3.1  Pointer x0 with update      mode 1
#define REG_X1_U0 (34) //  r/w  32  XY MEMORY DSP v3.1  Pointer x1 with update      mode 0
#define REG_X1_U1 (35) //  r/w  32  XY MEMORY DSP v3.1  Pointer x1 with update      mode 1
#define REG_X2_U0 (36) //  r/w  32  XY MEMORY DSP v3.1  Pointer x2 with update      mode 0
#define REG_X2_U1 (37) //  r/w  32  XY MEMORY DSP v3.1 Pointer x2 with update mode 1
#define REG_X3_U0 (38) //  r/w  32  XY MEMORY DSP v3.1 Pointer x3 with update mode 0
#define REG_X3_U1 (39) //  r/w  32  XY MEMORY DSP v3.1 Pointer x3 with update mode 1
#define REG_Y0_U0 (40) //  r/w  32  XY MEMORY DSP v3.1 Pointer y0 with update mode 0
#define REG_Y0_U1 (41) //  r/w  32  XY MEMORY DSP v3.1 Pointer y0 with update mode 1
#define REG_Y1_U0 (42) //  r/w  32  XY MEMORY DSP v3.1 Pointer y1 with update mode 0
#define REG_Y1_U1 (43) //  r/w  32  XY MEMORY DSP v3.1 Pointer y2 with update mode 1
#define REG_Y2_U0 (44) //  r/w  32  XY MEMORY DSP v3.1 Pointer y2 with update mode 0
#define REG_Y2_U1 (45) //  r/w  32  XY MEMORY DSP v3.1 Pointer y3 with update mode 1
#define REG_Y3_U0 (46) //  r/w  32  XY MEMORY DSP v3.1 Pointer y3 with update mode 0
#define REG_Y3_U1 (47) //  r/w  32  XY MEMORY DSP v3.1 Pointer y3 with update mode 1
#define REG_X0_NU (48) //  r/w  32  XY MEMORY DSP v3.1 Pointer x0 with no update
#define REG_X1_NU (49) //  r/w  32  XY MEMORY DSP v3.1 Pointer x1 with no update
#define REG_X2_NU (50) //  r/w  32  XY MEMORY DSP v3.1 Pointer x2 with no update
#define REG_X3_NU (51) //  r/w  32  XY MEMORY DSP v3.1 Pointer x3 with no update
#define REG_Y0_NU (52) //  r/w  32  XY MEMORY DSP v3.1 Pointer y0 with no update
#define REG_Y1_NU (53) //  r/w  32  XY MEMORY DSP v3.1 Pointer y1 with no update
#define REG_Y2_NU (54) //  r/w  32  XY MEMORY DSP v3.1 Pointer y2 with no update
#define REG_Y3_NU (55) //  r/w  32  XY MEMORY DSP v3.1 Pointer y3 with no update
#define REG_ACC1 (56) //  r/w  32  XMAC DSP v3.1 XMAC: Accumulator 1  saturating
#define REG_ACC2 (57) //  r/w  32  XMAC DSP v3.1 XMAC: Accumulator 2 =saturating
#ifndef __Xmult32d
#define REG_MLO (57)  //  r  32  MUL  Multiply Result low 32 bits   hcac compiler option -Xmult32d maps to R58
#define REG_MMID (58)  //  r  32  MUL  Multiply middle Result 32 bits hcac compiler option -Xmult32d removes
#else
#define REG_MLO (58)   //  r  32  MUL  Multiply Result low 32 bits  hcac compiler option -Xmult32d maps to R58
#endif
#define REG_MHI (59)   //  r  32  MUL  Multiply high Result 32 bits
#define REG_LP_COUNT  (60) // r  32  Basecase  CANNOT BE RE-ALLOCATED
// R61  reserved  - SHIMM indicator Basecase  CANNOT BE RE-ALLOCATED
// R62 reserved - LIMM data indicator r  32  Basecase  CANNOT BE RE-ALLOCATED
#define REG_PCL  (63)   //  r  30  Basecase  Program Counter, 32-   bit aligned

// EM XY registers
#if defined __Xxy && defined __Xagu && _ARCVER >= 0x40

// R32..35
#define AGU_U0 __AGU_U0
#define AGU_U1 __AGU_U1
#define AGU_U2 __AGU_U2
#define AGU_U3 __AGU_U3

#if defined(__Xagu_medium) || defined(__Xagu_large)

// R36..43
#define AGU_U4 __AGU_U4
#define AGU_U5 __AGU_U5
#define AGU_U6 __AGU_U6
#define AGU_U7 __AGU_U7
#define AGU_U8 __AGU_U8
#define AGU_U9 __AGU_U9
#define AGU_U10 __AGU_U10
#define AGU_U11 __AGU_U11

#if defined(__Xagu_large)

// R44..55
#define AGU_U12 __AGU_U12
#define AGU_U13 __AGU_U13
#define AGU_U14 __AGU_U14
#define AGU_U15 __AGU_U15
#define AGU_U16 __AGU_U16
#define AGU_U17 __AGU_U17
#define AGU_U18 __AGU_U18
#define AGU_U19 __AGU_U19
#define AGU_U20 __AGU_U20
#define AGU_U21 __AGU_U21
#define AGU_U22 __AGU_U22
#define AGU_U23 __AGU_U23

#endif // defined(__Xagu_large)
#endif // defined(__Xagu_medium) || defined(__Xagu_large)
#endif // defined __Xxy && defined __Xagu && _ARCVER >= 0x40

//  Auxiliary Register Allocation

//  There are a limited number of auxiliary register locations that allow short range
//  addressing using a signed 12-bit address. The diagram below shows an example
//  memory map of the recommended address allocations.
//  Auxiliary Register Address Usage
//  (-1) 0xFFFFFFFF (-2048) 0xFFFFF800  Customer Short Range Allocation - ARCompact (ARC600/ARC700/A5)
//  0xFFFFF7FF-0x80000000  Customer Allocation  - ARCompact (ARC600/ARC700/A5)
//  (-1) 0xFFFFFFFF (-512) 0xFFFFFE00  Customer Short Range Allocation - ARCtangent-A4/A3
//  0xFFFFFDFF-0x80000000  Customer Allocation  - ARCtangent-A4/A3
//  0x7FFFFFFF-0x00000000  ARC Allocation
//  (2047) 0x000007FF-0x00000000  ARC Short Range Allocation  - ARCompact (ARC600/ARC700/A5)
//  (512) 0x000001FF-0x00000000  ARC Short Range Allocation - ARCtangent-A4/A3
//  Recommended Auxiliary Register Usage
//  ARC International generally allocates from the lowest available free slot,
//  continuing upwards. It is recommended that customers start at the highest
//  available slot and work downwards. Customers can use the ARC allocated areas
//  for their own extension auxiliary registers. However, careful analysis of specific
//  register locations as used by ARC needs to be carried out before use.
//  Reserved registers are likely to be used by ARC for any future releases of the
//  product and should be avoided for customer extensions.
//  Unallocated registers within the ARC allocation address range that have been
//  marked as "not allocated" means that for this current release the location is not
//  used by ARC, but may be used in a future releases. "Not allocated" positions
//  should be used with care if the customer is considering putting their own
//  extensions in the ARC recommended address range.


#ifndef _AUX_DEFINES_NOT_WANTED

#define STATUS               0x0000
        /* Symbol: STATUS
           Component: Core
           Description: (obsolete)  Status Register
         */
#define SEMAPHORE            0x0001
        /* Symbol: SEMAPHORE
           Component: Core
           Description: Inter-process/Host semaphore register
         */
#define LP_START             0x0002
        /* Symbol: LP_START
           Component: Core
           Description: Loop Start Address
         */
#define LP_END               0x0003
        /* Symbol: LP_END
           Component: Core
           Description: Loop End Address
         */
#define IDENTITY             0x0004
        /* Symbol: IDENTITY
           Component: Core
           Description: Processor Identity Register
         */
#define DEBUG                0x0005
        /* Symbol: DEBUG
           Component: Core
           Description: Debug Register
         */
#define PC                   0x0006
        /* Symbol: PC
           Component: ARCompact V1/2
           Description: Program Counter
         */
#define ADCR                 0x0007
        /* Symbol: ADCR
           Component: Actionpoints
           Description:
            The data value used for comparing to the ARC
            instructions data or addresses for debugging
            NOTE: Optional on A4 & A5 only
         */
#define APCR                 0x0008
        /* Symbol: APCR
           Component: Actionpoints
           Description:
            The address returned for the point at which the
            actionpoint was set
            NOTE: Optional on A4 & A5 only
         */
#define ACR                  0x0009
        /* Symbol: ACR
           Component: Actionpoints
           Description:
            The control register for setting actionpoints in
            different modes of operation
            NOTE: Optional on A4 & A5 only
         */
#define STATUS32             0x000a
        /* Component: ARCompact V1/2
           Description: Status Register
           Access: R, writable with FLAG and KFLAG
         */
#define STATUS32_L1          0x000b
        /* Component: ARCompact V1 Core
           Description: Status Register save for level 1
           Access: RW
         */
#define STATUS32_P0          0x000b
        /* Component: AV2 Core
           Description: Status register priority 0
           Access: RW
         */
#define STATUS32_L2          0x000c
        /* Component: ARCompact V1 Core
           Description: Status Register save for level 2
           Access: RW
         */
#define AUX_USER_SP          0x000d
#define USER_SP             ((void*)AUX_USER_SP)
        /* Component: AV2 Core
           Description: Saved User mode stack pointer
           Access: RW
         */
#define AUX_IRQ_CONTROL      0x000e
        /* Component: AV2 Core
           Description: Interrupt context saving control register
           Access: RW
         */
#define BPU_FLUSH            0x000f
        /* Symbol: BPU_FLUSH
           Component: Core
           Description:
            Read/Write register to cause a hardware flush of the BPU
            NOTE: This is an ARC700 only aux register
         */
#define IVIC                 0x0010
#define IC_IVIC              0x0010
        /* Symbol: IVIC (or IC_IVIC)
           Component: Instruction Cache controller Multi-Way Icache
           Description:
            invalidate cache
         */
#define CHE_MODE             0x0011
#define IC_CTRL              0x0011
        /* Symbol: CHE_MODE (or IC_CTRL)
           Component: Instruction Cache controller Multi-Way Icache
           Description:
            cache control
         */
#define MULHI                0x0012
        /* Symbol: MULHI
           Component: MUL
           Description:
            High part of multiply to restore multiply state
            NOTE: Not available on the ARC700
         */
#define LOCKLINE             0x0013
#define IC_LIL               0x0013
        /* Symbol: LOCKLINE (or IC_LIL)
           Component: Cache Controller Multi-Way ICache
           Description:
            Lock line at this address cache lock instruction line
         */
#define DMC_CODE_RAM         0x0014
        /* Symbol: DMC_CODE_RAM
           Component: Cache Controller
           Description:
            Start address of DMC code RAM
            NOTE: Not available on the ARC700
         */
#define TAG_ADDR_MASK        0x0015
        /* Symbol: TAG_ADDR_MASK
           Component: Virtual Cache
           Description:
            Mask for tag address
            NOTE: Available on A4 only
         */
#define TAG_DATA_MASK        0x0016
        /* Symbol: TAG_DATA_MASK
           Component: Virtual Cache
           Description:
            Mask for tag data
            NOTE: Available on A4 only
         */
#define LINE_LENGTH_MASK     0x0017
        /* Symbol: LINE_LENGTH_MASK
           Component: Virtual Cache
           Description:
            Mask for line length counter
            NOTE: Available on A4 only
         */
#define AUX_LDST_RAM         0x0018
#define AUX_DCCM             0x0018
        /* Symbol: AUX_LDST_RAM (or AUX_DCCM)
           Component: Load Store Ram
           Description:
            Address of local load store RAM
            NOTE: Not available on the ARC700
         */
#define UNLOCKLINE           0x0019
#define IC_IVIL              0x0019
        /* Symbol: UNLOCKLINE (or IC_IVIL)
           Component: Cache Controller Multi-Way ICache
           Description:
            Unlock line at this address
            Cache invalidate instruction line
         */
#define IC_RAM_ADDRESS       0x001a
        /* Symbol: IC_RAM_ADDRESS
           Component: Multi-Way Icache
           Description:
            Cache ram access address
         */
#define IC_TAG               0x001b
        /* Symbol: IC_TAG
           Component: Multi-Way Icache
           Description:
            Tag ram access
         */
#define IC_WP                0x001c
        /* Symbol: IC_WP
           Component: Multi-Way Icache
           Description:
            Way pointer access
         */
#define IC_DATA              0x001d
        /* Symbol: IC_DATA
           Component: Multi-Way Icache
           Description:
            Multi-Way Icache data ram access
         */
#define SRAM_SEQ             0x0020
        /* Symbol: SRAM_SEQ
           Component: SRAM Sequencer
           Description:
            SRAM sequencer mode control
            NOTE: Not available on the ARC700
         */
#define COUNT                0x0021
        /* Symbol: COUNT
           Component: Enhanced Timer 0
           Description:
            Timer 0 Count value
         */
#define CONTROL              0x0022
        /* Symbol: CONTROL
           Component: Enhanced Timer 0
           Description:
            Timer 0 Control value
         */
#define LIMIT                0x0023
        /* Symbol: LIMIT
           Component: Enhanced Timer 0
           Description:
            Timer 0 Limit value
         */
#define PCPORT               0x0024
        /* Symbol: PCPORT
           Component: PC PORT
           Description:
            Control for PC_SEL line
         */
#define INT_VECTOR_BASE      ((void*)0x0025)
        /* Component: ARCompact V1/2 Core
           Description: Interrupt Vector Base address
           Access: RW
         */

#ifdef __Xvbfdw
#define AUX_VBFDW_MODE       0x0026
        /* Symbol: AUX_VBFDW_MODE
           Component: Viterbi Instruction
           Description:
            DVBF Instruction Mode
            NOTE: Not available on A4 or ARC700
         */
#define AUX_VBFDW_BM0        0x0027
        /* Symbol: AUX_VBFDW_BM0
           Component: Viterbi Instruction
           Description:
            Branch Metric 0
            NOTE: Not available on A4 or ARC700
         */
#define AUX_VBFDW_BM1        0x0028
        /* Symbol: AUX_VBFDW_BM1
           Component: Viterbi Instruction
           Description:
            Branch Metric 1
            NOTE: Not available on A4 or ARC700
         */
#define AUX_VBFDW_ACCU       0x0029
        /* Symbol: AUX_VBFDW_ACCU
           Component: Viterbi Instruction
           Description:
            DVBF Accumulator
            NOTE: Not available on A4 or ARC700
         */
#define AUX_VBFDW_OFST       0x002a
        /* Symbol: AUX_VBFDW_OFST
           Component: Viterbi Instruction
           Description:
            XY Memory path metric offset
            NOTE: Not available on A4 or ARC700
         */
#define AUX_VBFDW_INTSTAT    0x002b
        /* Symbol: AUX_VBFDW_INTSTAT
           Component: Viterbi Instruction
           Description:
            DVBF Internal State
            NOTE: Not available on A4 or ARC700
         */
#endif /* __Xvbfdw */

#ifdef __Xxy
#define AX2_A4               0x002c /* A4 */
#define AUX_XMAC0_24         0x002c /* A5, ARC600 */
        /* Symbol: AX2_A4 or AUX_XMAC0_24
           Component: XY Memory DSP v2.1 (A4)
                      XMAC DSP V3.1 (A5 - ARC600)
           Description:
            ADDRESS REGISTER FOR X2 (application note extension ) (A4)
            Accumulator XTP (A5 - ARC600)
         */
#define AY2_A4               0x002d /* A4 */
#define AUX_XMAC1_24         0x002d /* A5, ARC600 */
        /* Symbol: AY2_A4 or AUX_XMAC1_24
           Component: XY Memory DSP v2.1 (A4)
                      XMAC DSP V3.1 (A5 - ARC600)
           Description:
            ADDRESS REGISTER FOR Y2(application note extension ) (A4)
            Accumulator MSP (A5 - ARC600)
         */
#define MX2                  0x002e /* A4 */
#define AUX_XMAC2_24         0x002e /* A5, ARC600 */
        /* Symbol: MX2
           Component: XY Memory DSP v2.1 (A4)
                      XMAC DSP V3.1 (A5 - ARC600)
           Description:
            ADDRESS MODIFIER REGISTER FOR X2 (application note extension) (A4)
            Accumulator LSP (A5 - ARC600)
         */
#define MY2                  0x002f /* A4 */
#define AUX_FBF_STORE_16     0x002f /* A5, ARC600 */
        /* Symbol: MY2
           Component: XY Memory DSP v2.1 (A4)
                      XMAC FBF DSP V3.1 (A5 - ARC600)
           Description:
            ADDRESS MODIFIER REGISTER FOR Y2 application note extension) (A4)
            FBF OPERAND STORAGE (A5 - ARC600)
         */
#define AX0_A4               0x0030 /* A4 */
        /* Symbol: AX0_A4
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS REGISTER FOR X0
            NOTE: A4 only
         */
#define AX1_A4               0x0031 /* A4 */
        /* Symbol: AX1_A4
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS REGISTER FOR X1
            NOTE: A4 only
         */
#define AY0_A4               0x0032 /* A4 */
#define AUX_CRC_POLY         0x0032 /* (A5 - ARC600) */
        /* Symbol: AY0_A4 or AUX_CRC_POLY
           Component: XY MEMORY DSP v2.1 (A4)
                      Programmable CRC (A5 - ARC600)
           Description:
            ADDRESS REGISTER FOR Y0 (A4)
            POLYNOMIAL REGISTER (A5 - ARC600)
         */
#define AY1_A4               0x0033 /* A4 */
#define AUX_CRC_MODE         0x0033 /* (A5 - ARC600) */
        /* Symbol: AY1_A4
           Component: XY MEMORY DSP v2.1 (A4)
                      Programmable CRC (A5 - ARC600)
           Description:
            ADDRESS REGISTER FOR Y1 (A4)
            MODE REGISTER (A5 - ARC600)
         */
#define MX0                  0x0034
        /* Symbol: MX0
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS MODIFIER REGISTER FOR X0
            NOTE: A4 only
         */
#define MX1                  0x0035
        /* Symbol: MX1
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS MODIFIER REGISTER FOR X1
            NOTE: A4 only
         */
#define MY0                  0x0036
        /* Symbol: MY0
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS MODIFIER REGISTER FOR Y0
            NOTE: A4 only
         */
#define MY1                  0x0037
        /* Symbol: MY1
           Component: XY Memory DSP v2.1
           Description:
            ADDRESS MODIFIER REGISTER FOR Y1
            NOTE: A4 only
         */
#define XYCONFIG_A4          0x0038
        /* Symbol: XYCONFIG_A4
           Component: XY Memory DSP v2.1
           Description:
            CONFIGURATION REGISTER FOR XYMEMORY
            NOTE: A4 only
         */
#define SCRATCH_A            0x0039
        /* Symbol: SCRATCH_A
           Component: Scratch Ram
           Description:
            Long word offset in scratch RAM for burst to
            start at (0 is bottom of RAM)
            NOTE: A4 only
         */
#define BURSTSYS_A4          0x003a
        /* Symbol: BURSTSYS_A4
           Component: XY Memory DSP v2.1
           Description:
            BURST SYSTEM ADDRESS
            NOTE: A4 only
         */
#define BURSTXYM_A4          0x003b
        /* Symbol: BURSTXYM_A4
           Component: XY Memory DSP v2.1
           Description:
            BURST XY MEMORY ADDRESS
            NOTE: A4 only
         */
#define BURSTSZ_A4           0x003c
        /* Symbol: BURSTSZ_A4
           Component: XY Memory DSP v2.1
           Description:
            Burst size and dest register
            NOTE: A4 only
         */
#define BURSTVAL_A4          0x003d
        /* Symbol: BURSTVAL_A4
           Component: XY Memory DSP v2.1
           Description:
            Burst fill value register
            NOTE: A4 only
         */
#endif /* __Xxy */

#define SEP_CTRL          0x003f
        /* Component: V2EM Safety Enhancement Package
           Description: SEP hardware control register
           Access: RW
         */

#ifdef __Xxmac
#define XTP_NEWVAL           0x0040
        /* Symbol: XTP_NEWVAL
           Component: MAC
           Description:
            The 4-bit extended product of the multiply-accumulate
            result
            NOTE: Not available on the ARC700
         */
#define AUX_MACMODE          0x0041
        /* Symbol: AUX_MACMODE
           Component: XMAC DSP v2.1/DSP v3.0
           Description:
            MAC status and mode
         */
#define LSP_NEWVAL           0x0042
        /* Symbol: LSP_NEWVAL
           Component: MAC
           Description:
            Low 32-bit register product
            NOTE: Not available on the ARC700
         */
#endif /* __Xxmac */

#define AUX_IRQ_ACT          ((void*)0x0043)
        /* Component: AV2 Core
           Description: Active Interrupts Regsiter
           Access: RW
         */
#define AUX_IRQ_LV12         0x0043
        /* Symbol: AUX_IRQ_LV12
           Component: ARCompact V1 Core
           Description: Interrupt Level Status register
           Access: RW
         */

#ifdef __Xxmac
#define AUX_XMAC0            0x0044
        /* Symbol: AUX_XMAC0
           Component: XMAC DSP v2.1/DSP v3.0
           Description:
            Accumulator Extended Product
         */
#define AUX_XMAC1            0x0045
        /* Symbol: AUX_XMAC1
           Component: XMAC DSP v2.1/DSP v3.0
           Description:
            Accumulator Most Significant Part
         */
#define AUX_XMAC2            0x0046
        /* Symbol: AUX_XMAC2
           Component: XMAC DSP v2.1/DSP v3.0
           Description:
            Accumulator Least Significant Part
         */
#endif /* __Xxmac */

#define DC_IVDC              0x0047
        /* Symbol: DC_IVDC
           Component: Data Cache
           Description:
            Invalidate Data Cache
         */
#define DC_CTRL              0x0048
        /* Symbol: DC_CTRL
           Component: Data Cache
           Description:
            Control register
         */
#define DC_LDL               0x0049
        /* Symbol: DC_LDL
           Component: Data Cache
           Description:
            Lock data line
         */
#define DC_IVDL              0x004a
        /* Symbol: DC_IVDL
           Component: Data Cache
           Description:
            Invalidate data line
         */
#define DC_FLSH              0x004b
        /* Symbol: DC_FLSH
           Component: Data Cache
           Description:
            Flush data cache
         */
#define DC_FLDL              0x004c
        /* Symbol: DC_FLDL
           Component: Data Cache
           Description:
            Flush data line
         */
#define HEXDATA              0x0050
        /* Symbol: HEXDATA
           Component: ARCangel 2
           Description:
            Data to be displayed on Hex displays
            NOTE: A4 only
         */
#define HEXCTRL              0x0051
        /* Symbol: HEXCTRL
           Component: ARCangel 2
           Description:
            Blank/latch control of hex displays
            NOTE: A4 only
         */
#define LED                  0x0052
        /* Symbol: LED
           Component: ARCangel 2
           Description:
            Data written to front panel LED
            NOTE: A4 only
         */
#define LCDINSTR             0x0053 /* A4 */
#define AUX_LCD_DATA         0x0053 /* A4 */
#define AUX_LCD_DATA         0x0053 /* (A5 - ARC600) */
        /* Symbol: LCDINSTR
           Component: ARCangel 2/3
           Description:
            Instruction to be sent to LCD module LCD display
            read and write data
            NOTE: Not available on ARC700
         */
#define LCDDATA              0x0054 /* A4 */
#define AUX_LCD_CMD          0x0054 /* A4 */
#define AUX_LCD_CNTRL        0x0054 /* (A5 - ARC600) */
        /* Symbol: LCDDATA
           Component: ARCangel 2/3
           Description:
            Data to be sent to LCD module LCD display control register
            NOTE: Not available on ARC700
         */
#define LCDSTAT              0x0055 /* A4 */
#define AUX_US_COUNT         0x0055 /* (A5 - ARC600) */
        /* Symbol: LCDSTAT
           Component: ARCangel 2/3
           Description:
            Status indicator for LCD module LCD working frequency
            NOTE: Not available on ARC700
         */
#define DILSTAT              0x0056 /* A4 */
        /* Symbol: DILSTAT
           Component: ARCangel 2
           Description:
            Status indicator for DIL switches
            NOTE: A4 only
         */
#define SWSTAT               0x0057
#define AUX_LCD_BUTTON_REGISTER 0x0057
        /* Symbol: SWSTAT
           Component: ARCangel 2/3
           Description:
            Status indicator for front panel switches Front panel
            button auxiliary register
            NOTE: Not available on ARC700
         */
#define DC_RAM_ADDR          0x0058
        /* Symbol: DC_RAM_ADDR
           Component: Data Cache
           Description:
            Address for data cache RAM access
         */
#define DC_TAG               0x0059
        /* Symbol: DC_TAG
           Component: Data Cache
           Description:
            Tag data associated with dc_ram_addr
         */
#define DC_WP                0x005a
        /* Symbol: DC_WP
           Component: Data Cache
           Description:
            Way Pointer value associated with dc_ram_addr
         */
#define DC_DATA              0x005b
        /* Symbol: DC_DATA
           Component: Data Cache
           Description:
            Data longword associated with dc_ram_addr
         */
#define BCR_VER              0x0060
        /* Symbol: BCR_VER
           Component: Core
           Description:
            Build Configuration Registers Version
            NOTE: Not available on A4 or A5
         */
#define DCCM_BASE_BUILD      0x0061
        /* Symbol: DCCM_BASE_BUILD
           Component: DCCM
           Description:
            Base Address for DCCM
            NOTE: ARC700 only
         */
#define CRC_BUILD_BCR        0x0062
        /* Symbol: CRC_BUILD_BCR
           Component: CRC instruction
           Description:
            BCR for CRC Instruction Extension
         */
#define BTA_LINK_BUILD       0x0063
        /* Symbol: BTA_LINK_BUILD
           Component: BTA
           Description:
            Branch Target Address Link Register
            NOTE: ARC700 only
         */
#define DVBF_BUILD           0x0064
        /* Symbol: DVBF_BUILD
           Component: Dual viterbi butterfly instruction
           Description:
            Viterbi Build Register
         */
#define EA_BUILD             0x0065
        /* Symbol: EA_BUILD
           Component: Extended Arith
           Description:
            Extended Arithmetick Package Build Register
         */
#define DATASPACE            0x0066
        /* Symbol: DATASPACE
           Component: Dataspace
           Description:
            Dataspace
         */
#define MEMSUBSYS            0x0067
        /* Symbol: MEMSUBSYS
           Component: Memory subsystem
           Description:
            Memory subsystem
         */
#define VECBASE_AC_BUILD     0x0068
        /* Symbol: VECBASE_AC_BUILD
           Component: Core
           Description:
            ARCompact Interrupt Vector Base Address
         */
#define P_BASE_ADDR          0x0069
        /* Symbol: P_BASE_ADDR
           Component: Core
           Description:
            Peripheral base address
         */
#define DATA_UNCACHED_BUILD  0x006a
        /* Symbol: DATA_UNCACHED_BUILD
           Component: MMU
           Description:
            Data Uncached Build Configuration Register
         */
#define FP_BUILD             0x006b
        /* Symbol: FP_BUILD
           Component: Floating Point Unit
           Description:
            Build Configuration Register
         */
#define DPFP_BUILD           0x006c
        /* Symbol: DPFP_BUILD
           Component: Double Precision Floating Point Unit
           Description:
            Build Configuration Register for DPFP
         */
#define MPU_BUILD            0x006d
        /* Symbol: MPU_BUILD
           Component: MPU
           Description:
            MPU Build Register
         */
#define RF_BUILD             0x006e
        /* Symbol: RF_BUILD
           Component: Core Register File
           Description:
            Build Configuration Register
         */
#define MMU_BUILD            0x006f
        /* Symbol: MMU_BUILD
           Component: MMU
           Description:
            Build Configuration Register
         */
#define AA2_BUILD            0x0070
        /* Symbol: AA2_BUILD
           Component: AA2
           Description:
            Build Configuration Register
         */
#define VECBASE_BUILD        0x0071
        /* Symbol: VECBASE_BUILD
           Component: Core
           Description:
            Core Interrupt Vector Base Address Register
         */
#define D_CACHE_BUILD        0x0072
        /* Symbol: D_CACHE_BUILD
           Component: Data Cache
           Description:
            Build Configuration Register
         */
#define MADI_BUILD           0x0073
        /* Symbol: MADI_BUILD
           Component: Multiple ARC Debug Interface
           Description:
            Build Configuration Register
         */
#define DCCM_BUILD           0x0074
#define LDSTRAM_BUILD        0x0074
        /* Symbol: DCCM_BUILD
           Component: LD/ST RAM
           Description:
            Build Configuration Register
         */
#define TIMER_BUILD          0x0075
        /* Symbol: TIMER_BUILD
           Component: Timer
           Description:
            Build Configuration Register
         */
#define AP_BUILD             0x0076
        /* Symbol: AP_BUILD
           Component: Actionpoints
           Description:
            Build Configuration Register
         */
#define I_CACHE_BUILD        0x0077
        /* Symbol: I_CACHE_BUILD
           Component: Instruction Cache
           Description:
            Instruction Cache Build Configuration Register
         */
#define ADDSUB_BUILD         0x0078 /* (A5 - A4) */
#define ICCM_BUILD           0x0078 /* (ARC700 - ARC600) */
        /* Symbol: ICCM_BUILD
           Component: Saturated add/sub (A5 - A4)
                      ICCM (ARC700)
           Description:
            Build Configuration Register
         */
#define DSPRAM_BUILD         0x0079
        /* Symbol: DSPRAM_BUILD
           Component: Scratchpad and xy memory
           Description:
            Build Configuration Register
         */
#define MAC_BUILD            0x007a
#ifdef __Xdsp
    #define DSP_BUILD            0x007a  // DSP BCR
#endif
        /* Symbol: MAC_BUILD
           Component: MUL/MAC
           Description:
            Build Configuration Register
         */
#define MULTIPLY_BUILD       0x007b
        /* Symbol: MULTIPLY_BUILD
           Component: Multiply
           Description:
            Build Configuration Register
         */
#define SWAP_BUILD           0x007c
        /* Symbol: SWAP_BUILD
           Component: Swap
           Description:
            Build Configuration Register
         */
#define NORM_BUILD           0x007d
        /* Symbol: NORM_BUILD
           Component: Normalise
           Description:
            Build Configuration Register
         */
#define MINMAX_BUILD         0x007e
        /* Symbol: MINMAX_BUILD
           Component: MIN/MAX
           Description:
            Build Configuration Register
         */
#define BARREL_BUILD         0x007f
        /* Symbol: BARREL_BUILD
           Component: Barrel shift
           Description:
            Build Configuration Register
         */

#ifdef __Xxy
#define AX0                  0x0080
        /* Symbol: AX0
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AX1                  0x0081
        /* Symbol: AX1
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AX2                  0x0082
        /* Symbol: AX2
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AX3                  0x0083
        /* Symbol: AX3
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AY0                  0x0084
        /* Symbol: AY0
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AY1                  0x0085
        /* Symbol: AY1
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AY2                  0x0086
        /* Symbol: AY2
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define AY3                  0x0087
        /* Symbol: AY3
           Component: XY MEMORY DSP v3.0
           Description:
            Address register
         */
#define MX00                 0x0088
        /* Symbol: MX00
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX01                 0x0089
        /* Symbol: MX01
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX10                 0x008a
        /* Symbol: MX10
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX11                 0x008b
        /* Symbol: MX11
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX20                 0x008c
        /* Symbol: MX20
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX21                 0x008d
        /* Symbol: MX21
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX30                 0x008e
        /* Symbol: MX30
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MX31                 0x008f
        /* Symbol: MX31
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY00                 0x0090
        /* Symbol: MY00
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY01                 0x0091
        /* Symbol: MY01
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY10                 0x0092
        /* Symbol: MY10
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY11                 0x0093
        /* Symbol: MY11
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY20                 0x0094
        /* Symbol: MY20
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY21                 0x0095
        /* Symbol: MY21
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY30                 0x0096
        /* Symbol: MY30
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
#define MY31                 0x0097
        /* Symbol: MY31
           Component: XY MEMORY DSP v3.0
           Description:
            Modifier register
         */
/*#define XYCONFIG             0x0098 ...defined in xy.h */
        /* Symbol: XYCONFIG
           Component: XY MEMORY DSP v3.0
           Description:
            Configuration register
         */
/*#define BURSTSYS             0x0099 ...defined in xy.h */
        /* Symbol: BURSTSYS
           Component: XY MEMORY DSP v3.0
           Description:
            Starting burst system address
         */
/*#define BURSTXYM             0x009a ...defined in xy.h */
        /* Symbol: BURSTXYM
           Component: XY MEMORY DSP v3.0
           Description:
            Starting burst XY memory address
         */
/*#define BURSTSZ              0x009b ...defined in xy.h */
        /* Symbol: BURSTSZ
           Component: XY MEMORY DSP v3.0
           Description:
            Burst size and destination register
         */
/*#define BURSTVAL             0x009c ...defined in xy.h */
        /* Symbol: BURSTVAL
           Component: XY MEMORY DSP v3.0
           Description:
            Burst fill value register
         */
#endif /* __Xxy */

#ifdef __Xsimd
#define SE_CTRL              0x00a1
        /* Symbol: SE_CTRL
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_STAT              0x00a2
        /* Symbol: SE_STAT
           Component: SIMD Extension
           Description:
            SIMD Extensions
         */
#define SE_ERR               0x00a3
        /* Symbol: SE_ERR
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_EADR              0x00a4
        /* Symbol: SE_EADR
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_SPC               0x00a5
        /* Symbol: SE_SPC
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SDM_BASE             0x00a6
        /* Symbol: SDM_BASE
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SCM_BASE             0x00a7
        /* Symbol: SCM_BASE
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_DBG_CTRL          0x00a8
        /* Symbol: SE_DBG_CTRL
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_DBG_DATA0         0x00a9
        /* Symbol: SE_DBG_DATA0
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_DBG_DATA1         0x00aa
        /* Symbol: SE_DBG_DATA1
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_DBG_DATA2         0x00ab
        /* Symbol: SE_DBG_DATA2
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_DBG_DATA3         0x00ac
        /* Symbol: SE_DBG_DATA3
           Component: SIMD Extension
           Description:
            SIMD Extenstions
         */
#define SE_WATCH             0x00ad
        /* Symbol: SE_WATCH
           Component: SIMD Extension
           Description:
            SIMD Watchpoint Register
         */
#endif /* __Xsimd */

#define ARC600_BUILD_CONFG   0x00c1 /* ARC600 only */
        /* Symbol: ARC600_BUILD_CONFG
           Component: Core
           Description:
            BCR for ARC600 vesion 0x25 and above
         */
#define ISA_CONFIG           0x00c1
        /* Symbol: ISA_CONFIG
           Component: AV2 Core
           Description:
            ISA options register for AV2
            NOTE: AV2 only
         */

#define SEP_BUILD          0x00c7
        /* Component: V2EM Safety Enhancement Package BCR
           Description: EM SEP Build Configuration Register
           Access: R
         */

#define IRQ_BUILD            0x00f3
        /* Symbol: IRQ_BUILD
           Component: AV2 Core
           Description:
            Interrupt build configuration register
            NOTE: AV2 only
         */
#define PCT_BUILD            0x00f5
        /* Symbol: PCT_BUILD
           Component: Performance Monitoring
           Description:
            Performance Counter BCR
         */
#define CC_BUILD             0x00f6
        /* Symbol: CC_BUILD
           Component: Performance Monitoring
           Description:
            Countable Conditions BCR
         */
#define PM_BCR               0x00f7
        /* Symbol: PM_BCR
           Component: Power Management Unit
           Description:
            Build Configuration Register for the
            Power Management Unit
         */
#define SCQ_SWITCH_BUILD     0x00f8
        /* Symbol: SCQ_SWITCH_BUILD
           Component: SCQ Switch
           Description:
            Build Configuration Register for the SCQ Switch
         */
#define VRAPTOR_BUILD        0x00f9
        /* Symbol: VRAPTOR_BUILD
           Component: VRaptor Platform
           Description:
            Build Configuration Register for the VRaptor Platform
         */
#define DMA_CONFIG           0x00fa
        /* Symbol: DMA_CONFIG
           Component: ARC SIMD Ext
           Description:
            SIMD Extenstions
         */
#define SIMD_CONFIG          0x00fb
        /* Symbol: SIMD_CONFIG
           Component: ARC SIMD Ext
           Description:
            SIMD Extensions
         */
#define VLC_BUILD            0x00fc
        /* Symbol: VLC_BUILD
           Component: Core?
           Description:
            ARC VLC Ext VLC Extension
         */
#define SIMD_DMA_BUILD       0x00fd
        /* Symbol: SIMD_DMA_BUILD
           Component: ARC SIMD Ext
           Description:
            SIMD Extensions
         */
#define IFETCH_QUEUE_BUILD   0x00fe
        /* Symbol: IFETCH_QUEUE_BUILD
           Component: IFQ
           Description:
            IFQ
         */
#define LPB_BUILD            0x00e9
        /* Symbol: LPB_BUILD
           Component: IFQ
           Description: Loop Buffer BCR
         */
#define SMART_BUILD          0x00ff
        /* Symbol: SMART_BUILD
           Component: Small Realtime Trace
           Description:
            SMART
         */
#define COUNT1               0x0100
        /* Symbol: COUNT1
           Component: Enhanced Timer 1
           Description:
            Timer 1 Count value
         */
#define CONTROL1             0x0101
        /* Symbol: CONTROL1
           Component: Enhanced Timer 1
           Description:
            Timer 1 Control value
         */
#define LIMIT1               0x0102
        /* Symbol: LIMIT1
           Component: Enhanced Timer 1
           Description:
            Timer 1 Limit value
         */
#define AUX_RTC_CTRL         0x0103
        /* Symbol: AUX_RTC_CONTROL
           Component: AV2 RTC_OPTION = 1
           Description:
            Real time counter control register
            NOTE: AV2 only
         */
#define AUX_RTC_LOW          0x0104
        /* Symbol: AUX_RTC_LOW
           Component: AV2 RTC_OPTION = 1
           Description:
            Real time counter value [31:0]
            NOTE: AV2 only
         */
#define AUX_RTC_HIGH         0x0105
        /* Symbol: AUX_RTC_HIGH
           Component: AV2 RTC_OPTION = 1
           Description:
            Real time counter value [63:32]
            NOTE: AV2 only
         */

#define TIMER_XX_reservedF   0x0103
#define TIMER_XX_reservedL   0x011f /* 0x103-0x11f reserved */
        /* Symbol: TIMER_XX
           Component: Timers
           Description:
            Reserved for future timers
         */
#define ARCANGEL_PERIPH_XX_reservedF 0x0120
#define ARCANGEL_PERIPH_XX_reservedL 0x013f /* 0x120-0x13f reserved */
        /* Symbol: ARCANGEL_PERIPH_XX
           Component: ARCAngel Peripherals
           Description:
            Reserved for ARCAngel peripherals
         */
#define PERIPH_XX_reservedF  0x0140
#define PERIPH_XX_reservedL  0x01ff
        /* Symbol: PERIPH_XX
           Component: Peripherals
           Description:
            Reserved for peripherals (UARTS etc)
         */
#define IRQ_LEVEL_PENDING    0x0200  /* deprecated name */
#define IRQ_PRIORITY_PENDING    0x0200
        /* Component: AV2 Core
           Description: Interrupt Priority Pending Register
           Access: R
         */
#define AUX_IRQ_LEV          0x0200
        /* Component: ARCompact V1 Core
           Description: Interrupt Level Programming Register
           Access: RW
         */
#define AUX_IRQ_HINT         0x0201
        /* Component: ARCompact V1/2 cores
           Description: Software interrupt trigger
           Access: RW
         */
#define IRQ_PRIORITY         ((void*)0x0206)
        /* Component: AV2 Core
           Description: Interrupt Priority Register (banked by priority level)
           Access: RW
         */
#define IRQ_LEVEL            0x0207
        /* Component: AV2 Core
           Description:
            Interrupt level register
            NOTE: AV2 only
         */

#define AES_AUX_0            0x0210 /* reserved for IPSec */
#define AES_AUX_1            0x0211 /* reserved for IPSec */
#define AES_AUX_2            0x0212 /* reserved for IPSec */
#define AES_AUXS             0x0214 /* reserved for IPSec */
#define AES_AUXI             0x0215 /* reserved for IPSec */
#define AES_AUX_3            0x0216 /* reserved for IPSec */
#define AES_AUX_4            0x0217 /* reserved for IPSec */
#define ARITH_CTL_AUX        0x0218 /* reserved for IPSec */
#define DES_AUX              0x0219 /* reserved for IPSec */
        /* Symbol: <various>
           Component: IPSec
           Description:
            These are reserved for IPSec
         */
#define AP_AMV0              0x0220
        /* Symbol: AP_AMV0
           Component: Actionpoints
           Description:
            Actionpoint 0 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM0              0x0221
        /* Symbol: AP_AMM0
           Component: Actionpoints
           Description:
            Actionpoint 0 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC0               0x0222
        /* Symbol: AP_AC0
           Component: Actionpoints
           Description:
            Actionpoint 0 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV1              0x0223
        /* Symbol: AP_AMV1
           Component: Actionpoints
           Description:
            Actionpoint 1 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM1              0x0224
        /* Symbol: AP_AMM1
           Component: Actionpoints
           Description:
            Actionpoint 1 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC1               0x0225
        /* Symbol: AP_AC1
           Component: Actionpoints
           Description:
            Actionpoint 1 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV2              0x0226
        /* Symbol: AP_AMV2
           Component: Actionpoints
           Description:
            Actionpoint 2 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM2              0x0227
        /* Symbol: AP_AMM2
           Component: Actionpoints
           Description:
            Actionpoint 2 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC2               0x0228
        /* Symbol: AP_AC2
           Component: Actionpoints
           Description:
            Actionpoint 2 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV3              0x0229
        /* Symbol: AP_AMV3
           Component: Actionpoints
           Description:
            Actionpoint 3 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM3              0x022a
        /* Symbol: AP_AMM3
           Component: Actionpoints
           Description:
            Actionpoint 3 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC3               0x022b
        /* Symbol: AP_AC3
           Component: Actionpoints
           Description:
            Actionpoint 3 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV4              0x022c
        /* Symbol: AP_AMV4
           Component: Actionpoints
           Description:
            Actionpoint 4 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM4              0x022d
        /* Symbol: AP_AMM4
           Component: Actionpoints
           Description:
            Actionpoint 4 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC4               0x022e
        /* Symbol: AP_AC4
           Component: Actionpoints
           Description:
            Actionpoint 4 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV5              0x022f
        /* Symbol: AP_AMV5
           Component: Actionpoints
           Description:
            Actionpoint 5 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM5              0x0230
        /* Symbol: AP_AMM5
           Component: Actionpoints
           Description:
            Actionpoint 5 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC5               0x0231
        /* Symbol: AP_AC5
           Component: Actionpoints
           Description:
            Actionpoint 5 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV6              0x0232
        /* Symbol: AP_AMV6
           Component: Actionpoints
           Description:
            Actionpoint 6 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM6              0x0233
        /* Symbol: AP_AMM6
           Component: Actionpoints
           Description:
            Actionpoint 6 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC6               0x0234
        /* Symbol: AP_AC6
           Component: Actionpoints
           Description:
            Actionpoint 6 Control
            NOTE: ARC600/700 only
         */
#define AP_AMV7              0x0235
        /* Symbol: AP_AMV7
           Component: Actionpoints
           Description:
            Actionpoint 7 Match Value
            NOTE: ARC600/700 only
         */
#define AP_AMM7              0x0236
        /* Symbol: AP_AMM7
           Component: Actionpoints
           Description:
            Actionpoint 7 Match Mask
            NOTE: ARC600/700 only
         */
#define AP_AC7               0x0237
        /* Symbol: AP_AC7
           Component: Actionpoints
           Description:
            Actionpoint 7 Control
            NOTE: ARC600/700 only
         */

#define WDT_PASSWD        0x0238
        /* Component: V2EM Safety Enhancement Package
           Description: Watchdog timer password register
           Access: W
         */
#define WDT_CTRL          0x0239
        /* Component: V2EM Safety Enhancement Package
           Description: Watchdog timer control register
           Access: RW
         */
#define WDT_PERIOD        0x023a
        /* Component: V2EM Safety Enhancement Package
           Description: Watchdog timer period register
           Access: RW
         */
#define WDT_COUNT         0x023b
        /* Component: V2EM Safety Enhancement Package
           Description: Watchdog timer count register
           Access: R
         */


#define CC_BASE              0x0240
#define CC_LAST              0x0243
        /* Symbol: CC_<BASE,LAST>
           Component: Performance Monitoring
           Description:
            Countable Conditions Discovery registers
            NOTE: ARC600/700 only
         */
#define PCT_COUNT_BASE       0x0250
#define PCT_COUNT_LAST       0x025f
        /* Symbol: PCT_COUNT_<BASE,LAST>
           Component: Performance Monitoring
           Description:
            Performance Counter Count Values
            NOTE: ARC600/700 only
         */
#define PCT_SNAP_BASE        0x0260
#define PCT_SNAP_LAST        0x026f
        /* Symbol: PCT_SNAP_<BASE,LAST>
           Component: Performance Monitoring
           Description:
            Performance Counter Snapshot Values
            NOTE: ARC600/700 only
         */
#define PCT_CONFIG_BASE      0x0270
#define PCT_CONFIG_LAST      0x0277
        /* Symbol: PCT_CONFIG_<BASE,LAST>
           Component: Performance Monitoring
           Description:
            Performance Counter Configuration Registers
            NOTE: ARC600/700 only
         */
#define PCT_CONTROL          0x0278
        /* Symbol: PCT_CONTROL
           Component: Performance Monitoring
           Description:
            Performance Counter Control Registers
            NOTE: ARC600/700 only
         */
#define PCT_BANK             0x0279
        /* Symbol: PCT_BANK
           Component: Performance Monitoring
           Description:
            Performance Counter Bank Registers
            NOTE: ARC600/700 only
         */
#define JLI_BASE             0x0290
        /* Component: AV2 CODE_DENSITY
           Description: Jump and Linked Indexed Base Address
         */
#define LDI_BASE             0x0291
        /* Component: AV2 CODE_DENSITY
           Description: Load Indexed Base Address
         */
#define EI_BASE              0x0292
        /* Component: AV2 CODE_DENSITY
           Description: Execute Indexed Base Address
         */
#define FP_STATUS            0x0300
        /* Symbol: FP_STATUS
           Component: FP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Floating Point Status Register (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define AUX_DPFP1L           0x0301
        /* Symbol: AUX_DPFP1L
           Component: FP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Double Precision Floating Point
             D1 Lower Reg (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define AUX_DPFP1H           0x0302
        /* Symbol: AUX_DPFP1H
           Component: FP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Double Precision Floating Point D1 Higher Reg
            (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define AUX_DPFP2L           0x0303
        /* Symbol: AUX_DPFP2L
           Component: FP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Double Precision Floating Point D2 Lower Reg
            (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define AUX_DPFP2H           0x0304
        /* Symbol: AUX_DPFP2H
           Component: FP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Double Precision Floating Point D2 Higher Reg
            (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define DPFP_STATUS          0x0305
        /* Symbol: DPFP_STATUS
           Component: DPFP Unit (ARC700 - ARC600)
                      Real-Time Trace (A5 - A4)
           Description:
            Double Precision Floating Point Status Reg
            (ARC700 - ARC600)
            RTT Reserved Registers (A5 - A4)
         */
#define RTT_BASE             0x0306
#define RTT_LAST             0x03ff
        /* Symbol: RTT_<BASE,LAST>
           Component: Real-Time Trace
           Description:
            RTT Reserved Registers
         */
#define ERET                 ((void*)0x0400)
        /* Component: ARC700, ARCV2
           Description: Exception Return Address
           Access: RW
         */
#define ERBTA                ((void*)0x0401)
        /* Component: ARC700, ARCV2
           Description: Exception Return Branch Target Address
           Access: RW
         */
#define ERSTATUS             ((void*)0x0402)
        /* Component: ARC700, ARCV2
           Description: Exception Return Status
           Access: RW
         */
#define ECR                  ((void*)0x0403)
        /* Component: ARC700, ARCV2
           Description: Exception Cause Register
           Access: RW
         */
#define EFA                  ((void*)0x0404)
        /* Component: ARC700, ARCV2
           Description: Exception Fault Address
           Access: RW
         */
#define TLBPD0               0x0405
        /* Symbol: TLBPD0
           Component: MMU
           Description:
            MMU TLB Page Descriptor register 0 (ARC700 only)
         */
#define TLBPD1               0x0406
        /* Symbol: TLBPD1
           Component: MMU
           Description:
            MMU TLB Page Descriptor register 1 (ARC700 only)
         */
#define TLBINDEX             0x0407
        /* Symbol: TLBINDEX
           Component: MMU
           Description:
            MMU TLB Index register (ARC700 only)
         */
#define TLBCOMMAND           0x0408
        /* Symbol: TLBCOMMAND
           Component: MMU
           Description:
            MMU TLB Command register (ARC700 only)
         */
#define PID                  0x0409 /* ARC700 */
#define MPU_ENABLE           0x0409 /* ARC600 */
        /* Symbol: PID or MPU_ENABLE
           Component: MMU (ARC700), MPU (ARC600)
           Description:
            MMU Process ID TLB enable (ARC700)
            MPU Enable Register (ARC600)
         */
#define ICAUSE               ((void*)0x040a)
        /* Component: AV2 Core
           Description: Interrupt cause register (banked by priority level)
           Access: R
         */
#define ICAUSE1              0x040a
        /* Component: ARC700
           Description: Level 1 Interrupt Cause Register
           Access: R
         */
#define ICAUSE2              0x040b
        /* Component: ARC700
           Description: Level 2 Interrupt Cause Register
           Access: R
         */
#define IRQ_INTERRUPT        ((void*)0x040b)  /* deprecated name */
#define IRQ_SELECT        0x040b
        /* Component: AV2 Core
           Description: Interrupt bank select register
           Access: RW
         */
#define IRQ_ENABLE           0x040c
        /* Component: AV2 Core
           Description: Interrupt enable Register (banked by priority level)
           Access: RW
         */
#define AUX_IENABLE          0x040c
        /* Component: ARC700
           Description: Interrupt Mask Programming
           Access: RW
         */
#define IRQ_TRIGGER          0x040d
        /* Component: AV2 Core
           Description: Interrupt Trigger Register (banked by priority level)
           Access: RW
         */
#define AUX_ITRIGGER         0x040d
        /* Component: ARC700
           Description: Interrupt Sensitivity Programming
           Access: RW
         */
#define IRQ_STATUS           0x040f
        /* Component: AV2 Core
           Description: Interrupt Status Register (banked by priority level)
           Access: R
         */
#define XPU                  0x0410
        /* Component: ARC700, ARCV2
           Description: User Mode Extension Enable Register
           Access: RW
         */
#define BTA                  0x0412
        /* Component: ARC700, ARCV2
           Description: Branch Target Address
           Access: RW
         */
#define BTA_L1               0x0413
        /* Component: ARC700
           Description: Interrupt Branch Target Link Register (Level 1)
           Access: RW
         */
#define BTA_L2               0x0414
        /* Component: ARC700
           Description: Interrupt Branch Target Link Register (Level 2)
           Access: RW
         */
#define IRQ_PULSE_CANCEL     0x0415
        /* Component: AV2 Core
           Description: Interrupt Pulse Cancel Register (banked by priority level)
           Access: W
         */
#define AUX_IRQ_PULSE_CANCEL 0x0415
        /* Component: ARC700
           Description: Interrupt Pulse Cancel Register
           Access: W
         */
#define IRQ_PENDING      0x0416
        /* Component: AV2 Core
           Description: Interrupt Pending Register (banked by priority level)
           Access: R
         */
#define AUX_IRQ_PENDING      0x0416
        /* Component: ARC700
           Description: Interrupt Pending Register
           Access: R
         */
#define SCRATCH_DATA0        0x0418
        /* Symbol: SCRATCH_DATA0
           Component: MMU
           Description:
            32-bit scratch auxiliary register (ARC700 only)
         */
#define MPU_IC               0x0420
        /* Symbol: MPU_IC
           Component: MPU
           Description:
            MPU interrupt cause register (ARC600 only)
         */
#define MPU_FAULT            0x0421
        /* Symbol: MPU_FAULT
           Component: MPU
           Description:
            MPU fault address register (ARC600 only)
         */
#define MPU_RDB0             0x0422
        /* Symbol: MPU_RDB0
           Component: MPU
           Description:
            MPU Region Descriptor Base 0 (ARC600 only)
         */
#define MPU_RDP0             0x0423
        /* Symbol: MPU_RDP0
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 0 (ARC600 only)
         */
#define MPU_RDB1             0x0424
        /* Symbol: MPU_RDB1
           Component: MPU
           Description:
            MPU Region Descriptor Base 1 (ARC600 only)
         */
#define MPU_RDP1             0x0425
        /* Symbol: MPU_RDP1
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 1 (ARC600 only)
         */
#define MPU_RDB2             0x0426
        /* Symbol: MPU_RDB2
           Component: MPU
           Description:
            MPU Region Descriptor Base 2 (ARC600 only)
         */
#define MPU_RDP2             0x0427
        /* Symbol: MPU_RDP2
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 2 (ARC600 only)
         */
#define MPU_RDB3             0x0428
        /* Symbol: MPU_RDB3
           Component: MPU
           Description:
            MPU Region Descriptor Base 3 (ARC600 only)
         */
#define MPU_RDP3             0x0429
        /* Symbol: MPU_RDP3
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 3 (ARC600 only)
         */
#define MPU_RDB4             0x042a
        /* Symbol: MPU_RDB4
           Component: MPU
           Description:
            MPU Region Descriptor Base 4 (ARC600 only)
         */
#define MPU_RDP4             0x042b
        /* Symbol: MPU_RDP4
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 4 (ARC600 only)
         */
#define MPU_RDB5             0x042c
        /* Symbol: MPU_RDB5
           Component: MPU
           Description:
            MPU Region Descriptor Base 5 (ARC600 only)
         */
#define MPU_RDP5             0x042d
        /* Symbol: MPU_RDP5
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 5 (ARC600 only)
         */
#define MPU_RDB6             0x042e
        /* Symbol: MPU_RDB6
           Component: MPU
           Description:
            MPU Region Descriptor Base 6 (ARC600 only)
         */
#define MPU_RDP6             0x042f
        /* Symbol: MPU_RDP6
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 6 (ARC600 only)
         */
#define MPU_RDB7             0x0430
        /* Symbol: MPU_RDB7
           Component: MPU
           Description:
            MPU Region Descriptor Base 7 (ARC600 only)
         */
#define MPU_RDP7             0x0431
        /* Symbol: MPU_RDP7
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 7 (ARC600 only)
         */
#define MPU_RDB8             0x0432
        /* Symbol: MPU_RDB8
           Component: MPU
           Description:
            MPU Region Descriptor Base 8 (ARC600 only)
         */
#define MPU_RDP8             0x0433
        /* Symbol: MPU_RDP8
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 8 (ARC600 only)
         */
#define MPU_RDB9             0x0434
        /* Symbol: MPU_RDB9
           Component: MPU
           Description:
            MPU Region Descriptor Base 9 (ARC600 only)
         */
#define MPU_RDP9             0x0435
        /* Symbol: MPU_RDP9
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 9 (ARC600 only)
         */
#define MPU_RDB10            0x0436
        /* Symbol: MPU_RDB10
           Component: MPU
           Description:
            MPU Region Descriptor Base 10 (ARC600 only)
         */
#define MPU_RDP10            0x0437
        /* Symbol: MPU_RDP10
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 10 (ARC600 only)
         */
#define MPU_RDB11            0x0438
        /* Symbol: MPU_RDB11
           Component: MPU
           Description:
            MPU Region Descriptor Base 11 (ARC600 only)
         */
#define MPU_RDP11            0x0439
        /* Symbol: MPU_RDP11
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 11 (ARC600 only)
         */
#define MPU_RDB12            0x043a
        /* Symbol: MPU_RDB12
           Component: MPU
           Description:
            MPU Region Descriptor Base 12 (ARC600 only)
         */
#define MPU_RDP12            0x043b
        /* Symbol: MPU_RDP12
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 12 (ARC600 only)
         */
#define MPU_RDB13            0x043c
        /* Symbol: MPU_RDB13
           Component: MPU
           Description:
            MPU Region Descriptor Base 13 (ARC600 only)
         */
#define MPU_RDP13            0x043d
        /* Symbol: MPU_RDP13
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 13 (ARC600 only)
         */
#define MPU_RDB14            0x043e
        /* Symbol: MPU_RDB14
           Component: MPU
           Description:
            MPU Region Descriptor Base 14 (ARC600 only)
         */
#define MPU_RDP14            0x043f
        /* Symbol: MPU_RDP14
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 14 (ARC600 only)
         */
#define MPU_RDB15            0x0440
        /* Symbol: MPU_RDB15
           Component: MPU
           Description:
            MPU Region Descriptor Base 15 (ARC600 only)
         */
#define MPU_RDP15            0x0441
        /* Symbol: MPU_RDP15
           Component: MPU
           Description:
            MPU Region Descriptor Permissions 15 (ARC600 only)
         */
#define EIA_FLAGS            0x044f
        /* Symbol: EIA_FLAGS
           Component: EIA
           Description:
            Flags for EIA (ARC600/700 only)
         */
#define PM_STATUS            0x0450
        /* Symbol: PM_STATUS
           Component: PMU
           Description:
            PMU status register (ARC600/700 only)
         */
#define WAKE                 0x0451
        /* Symbol: WAKE
           Component: PMU
           Description:
            PMU timer wake register (ARC600/700 only)
         */
#define DVFS_PERFORMANCE     0x0452
        /* Symbol: DVFS_PERFORMANCE
           Component: PMU
           Description:
            PMU DVFS performance control register (ARC600/700 only)
         */
#define PWR_CTRL             0x0453
        /* Symbol: PWR_CTRL
           Component: PMU
           Description:
            PMU External component power control (ARC600/700 only)
         */

/* 0x454 - 0x459 Reserved for PMU development */

#define LPB_CTRL             0x0488
        /* Symbol: LPB_CTRL
           Component: IFQ`
           Description: Loop buffer control register (HS VDSP only)
         */


#define AUX_VLC_BUF_IDX      0x0500
        /* Symbol: AUX_VLC_BUF_IDX
           Component: VLC Ext
           Description:
            Bit stream Buffer Pointer for buffer read
         */
#define AUX_VLC_READ_BUF     0x0501
        /* Symbol: AUX_VLC_READ_BUF
           Component: VLC Ext
           Description:
            Bit stream Buffer Read register
         */
#define AUX_VLC_VALID_BITS   0x0502
        /* Symbol: AUX_VLC_VALID_BITS
           Component: VLC Ext
           Description:
            Bit stream Buffer Last Item Total Valid
            Bits
         */
#define AUX_VLC_BUF_IN       0x0503
        /* Symbol: AUX_VLC_BUF_IN
           Component: VLC Ext
           Description:
            Bit stream Buffer Write Register
         */
#define AUX_VLC_BUF_FREE     0x0504
        /* Symbol: AUX_VLC_BUF_FREE
           Component: VLC Ext
           Description:
            Bit stream Buffer Number of free items
            space
         */
#define AUX_VLC_IBUF_STATUS  0x0505
        /* Symbol: AUX_VLC_IBUF_STATUS
           Component: VLC Ext
           Description:
            Bit stream Buffer Status. Write to
            change bit offset and Buffer tail pointer.
         */
#define AUX_VLC_SETUP        0x0506
        /* Symbol: AUX_VLC_SETUP
           Component: VLC Ext
           Description:
            Setup register: Decoder select and interrupt
            ctrl
         */
#define AUX_VLC_BITS         0x0507
        /* Symbol: AUX_VLC_BITS
           Component: VLC Ext
           Description:
            Returns the next 32-bits from the bit stream
         */
#define AUX_VLC_TABLE        0x0508
        /* Symbol: AUX_VLC_TABLE
           Component: VLC Ext
           Description:
            Table select
         */
#define AUX_VLC_GET_SYMBOL   0x0509
        /* Symbol: AUX_VLC_GET_SYMBOL
           Component: VLC Ext
           Description:
            Read decoded Symbol and update offset
         */
#define AUX_VLC_READ_SYMBOL  0x050a
        /* Symbol: AUX_VLC_READ_SYMBOL
           Component: VLC Ext
           Description:
            Read decoded Symbol only
         */
#define VLC_RESERVED_50B     0x050b
#define VLC_RESERVED_50F     0x050f
        /* Symbol: VLC_<RESERVED_50B,RESERVED_50F>
           Component: VLC Ext
           Description:
            VLC Ext Reserved
         */
#define AUX_UCAVLC_SETUP     0x0510
        /* Symbol: AUX_UCAVLC_SETUP
           Component: VLC Ext
           Description:
            UVLC and CAVLC Decode Setup Register
         */
#define AUX_UCAVLC_STATE     0x0511
        /* Symbol: AUX_UCAVLC_STATE
           Component: VLC Ext
           Description:
            UVLC and CAVLC Decode Intermediate State
            Values Registers
         */
#define AUX_CAVLC_ZERO_LEFT  0x0512
        /* Symbol: AUX_CAVLC_ZERO_LEFT
           Component: VLC Ext
           Description:
            CAVLC Zero Run decode Zero Left
            Register .
         */
#define VLC_RESERVED_513     0x0513
        /* Symbol: VLC_RESERVED_513
           Component: VLC Ext
           Description:
            Reserved
         */
#define AUX_UVLC_I_STATE     0x0514
        /* Symbol: AUX_UVLC_I_STATE
           Component: VLC Ext
           Description:
            UVLC Internal states for context
            saving /restoring.
         */
#define VLC_RESERVED_515     0x0515
#define VLC_RESERVED_51B     0x051b
        /* Symbol: VLC_<RESERVED_515,RESERVED_51B>
           Component: VLC Ext
           Description:
            VLC Ext Reserved
         */
#define AUX_VLC_DMA_PTR      0x051c
        /* Symbol: AUX_VLC_DMA_PTR
           Component: VLC Ext
           Description:
            Physical address of current DMA location
         */
#define AUX_VLC_DMA_END      0x051d
        /* Symbol: AUX_VLC_DMA_END
           Component: VLC Ext
           Description:
            Physical address of end of data to DMA
         */
#define AUX_VLC_DMA_ESC      0x051e
        /* Symbol: AUX_VLC_DMA_ESC
           Component: VLC Ext
           Description:
            Escape sequence in bit stream.
         */
#define AUX_VLC_DMA_CTRL     0x051f
        /* Symbol: AUX_VLC_DMA_CTRL
           Component: VLC Ext
           Description:
            DMA assist control register
         */
#define AUX_VLC_GET_0BIT     0x0520
        /* Symbol: AUX_VLC_GET_0BIT
           Component: VLC Ext
           Description:
            read and consume 0 bit from bit stream
         */
#define AUX_VLC_GET_1BIT     0x0521
        /* Symbol: AUX_VLC_GET_1BIT
           Component: VLC Ext
           Description:
            read and consume 1 bit from bit stream
         */
#define AUX_VLC_GET_2BIT     0x0522
        /* Symbol: AUX_VLC_GET_2BIT
           Component: VLC Ext
           Description:
            read and consume 2 bit from bit stream
         */
#define AUX_VLC_GET_3BIT     0x0523
        /* Symbol: AUX_VLC_GET_3BIT
           Component: VLC Ext
           Description:
            read and consume 3 bit from bit stream
         */
#define AUX_VLC_GET_4BIT     0x0524
        /* Symbol: AUX_VLC_GET_4BIT
           Component: VLC Ext
           Description:
            read and consume 4 bit from bit stream
         */
#define AUX_VLC_GET_5BIT     0x0525
        /* Symbol: AUX_VLC_GET_5BIT
           Component: VLC Ext
           Description:
            read and consume 5 bit from bit stream
         */
#define AUX_VLC_GET_6BIT     0x0526
        /* Symbol: AUX_VLC_GET_6BIT
           Component: VLC Ext
           Description:
            read and consume 6 bit from bit stream
         */
#define AUX_VLC_GET_7BIT     0x0527
        /* Symbol: AUX_VLC_GET_7BIT
           Component: VLC Ext
           Description:
            read and consume 7 bit from bit stream
         */
#define AUX_VLC_GET_8BIT     0x0528
        /* Symbol: AUX_VLC_GET_8BIT
           Component: VLC Ext
           Description:
            read and consume 8 bit from bit stream
         */
#define AUX_VLC_GET_9BIT     0x0529
        /* Symbol: AUX_VLC_GET_9BIT
           Component: VLC Ext
           Description:
            read and consume 9 bit from bit stream
         */
#define AUX_VLC_GET_10BIT    0x052a
        /* Symbol: AUX_VLC_GET_10BIT
           Component: VLC Ext
           Description:
            read and consume 10 bit from bit stream
         */
#define AUX_VLC_GET_11BIT    0x052b
        /* Symbol: AUX_VLC_GET_11BIT
           Component: VLC Ext
           Description:
            read and consume 11 bit from bit stream
         */
#define AUX_VLC_GET_12BIT    0x052c
        /* Symbol: AUX_VLC_GET_12BIT
           Component: VLC Ext
           Description:
            read and consume 12 bit from bit stream
         */
#define AUX_VLC_GET_13BIT    0x052d
        /* Symbol: AUX_VLC_GET_13BIT
           Component: VLC Ext
           Description:
            read and consume 13 bit from bit stream
         */
#define AUX_VLC_GET_14BIT    0x052e
        /* Symbol: AUX_VLC_GET_14BIT
           Component: VLC Ext
           Description:
            read and consume 14 bit from bit stream
         */
#define AUX_VLC_GET_15BIT    0x052f
        /* Symbol: AUX_VLC_GET_15BIT
           Component: VLC Ext
           Description:
            read and consume 15 bit from bit stream
         */
#define AUX_VLC_GET_16BIT    0x0530
        /* Symbol: AUX_VLC_GET_16BIT
           Component: VLC Ext
           Description:
            read and consume 16 bit from bit stream
         */
#define AUX_VLC_GET_17BIT    0x0531
        /* Symbol: AUX_VLC_GET_17BIT
           Component: VLC Ext
           Description:
            read and consume 17 bit from bit stream
         */
#define AUX_VLC_GET_18BIT    0x0532
        /* Symbol: AUX_VLC_GET_18BIT
           Component: VLC Ext
           Description:
            read and consume 18 bit from bit stream
         */
#define AUX_VLC_GET_19BIT    0x0533
        /* Symbol: AUX_VLC_GET_19BIT
           Component: VLC Ext
           Description:
            read and consume 19 bit from bit stream
         */
#define AUX_VLC_GET_20BIT    0x0534
        /* Symbol: AUX_VLC_GET_20BIT
           Component: VLC Ext
           Description:
            read and consume 20 bit from bit stream
         */
#define AUX_VLC_GET_21BIT    0x0535
        /* Symbol: AUX_VLC_GET_21BIT
           Component: VLC Ext
           Description:
            read and consume 21 bit from bit stream
         */
#define AUX_VLC_GET_22BIT    0x0536
        /* Symbol: AUX_VLC_GET_22BIT
           Component: VLC Ext
           Description:
            read and consume 22 bit from bit stream
         */
#define AUX_VLC_GET_23BIT    0x0537
        /* Symbol: AUX_VLC_GET_23BIT
           Component: VLC Ext
           Description:
            read and consume 23 bit from bit stream
         */
#define AUX_VLC_GET_24BIT    0x0538
        /* Symbol: AUX_VLC_GET_24BIT
           Component: VLC Ext
           Description:
            read and consume 24 bit from bit stream
         */
#define AUX_VLC_GET_25BIT    0x0539
        /* Symbol: AUX_VLC_GET_25BIT
           Component: VLC Ext
           Description:
            read and consume 25 bit from bit stream
         */
#define AUX_VLC_GET_26BIT    0x053a
        /* Symbol: AUX_VLC_GET_26BIT
           Component: VLC Ext
           Description:
            read and consume 26 bit from bit stream
         */
#define AUX_VLC_GET_27BIT    0x053b
        /* Symbol: AUX_VLC_GET_27BIT
           Component: VLC Ext
           Description:
            read and consume 27 bit from bit stream
         */
#define AUX_VLC_GET_28BIT    0x053c
        /* Symbol: AUX_VLC_GET_28BIT
           Component: VLC Ext
           Description:
            read and consume 28 bit from bit stream
         */
#define AUX_VLC_GET_29BIT    0x053d
        /* Symbol: AUX_VLC_GET_29BIT
           Component: VLC Ext
           Description:
            read and consume 29 bit from bit stream
         */
#define AUX_VLC_GET_30BIT    0x053e
        /* Symbol: AUX_VLC_GET_30BIT
           Component: VLC Ext
           Description:
            read and consume 30 bit from bit stream
         */
#define AUX_VLC_GET_31BIT    0x053f
        /* Symbol: AUX_VLC_GET_31BIT
           Component: VLC Ext
           Description:
            read and consume 31 bit from bit stream
         */
#define AUX_CABAC_CTRL       0x0540
        /* Symbol: AUX_CABAC_CTRL
           Component: VLC Ext
           Description:
            Main CABAC decoder control register
         */
#define AUX_CABAC_CTX_STATE  0x0541
        /* Symbol: AUX_CABAC_CTX_STATE
           Component: VLC Ext
           Description:
            Access to context table
         */
#define AUX_CABAC_COD_PARAM  0x0542
        /* Symbol: AUX_CABAC_COD_PARAM
           Component: VLC Ext
           Description:
            Arithmetic decoding engine parameters
         */
#define AUX_CABAC_MISC0      0x0543
        /* Symbol: AUX_CABAC_MISC0
           Component: VLC Ext
           Description:
            Additional control fields for some syntax
            elements
         */
#define AUX_CABAC_MISC1      0x0544
        /* Symbol: AUX_CABAC_MISC1
           Component: VLC Ext
           Description:
            Internal state (for context switching)
         */
#define AUX_CABAC_MISC2      0x0545
        /* Symbol: AUX_CABAC_MISC2
           Component: VLC Ext
           Description:
            Internal state (for context switching)
         */

#ifdef __Xdsp
#define ACC0_LO        0x580   // low accumulator register
#define ACC0_GLO       0x581   // low accumulator guard+status
#define ACC0_HI        0x582   // high accumulator register
#define ACC0_GHI       0x583   // high accumulator guard+status
#define DSP_BFLY       0x59d   // Holds A0 operand for complex
#define DSP_FFT_CTRL   0x59e   // DSP complex bufferfly control
#define DSP_CTRL       0x59f   // DSP control register
#endif

#define ARC601_BUILD_CONFIG  0x0600
        /* Symbol: ARC601_BUILD_CONFIG
           Component: Core
           Description:
            Build configuration register for ARC601 core
         */
#define SMART_CONTROL        0x0700
        /* Symbol: SMART_CONTROL
           Component: Small Realtime Trace
           Description:
            Control SmaRT Operation (ARC600/700 only)
         */
#define SMART_DATA_0         0x0701 /* ARC700 */
#define SMART_DATA           0x0701 /* ARC600 */
        /* Symbol: SMART_DATA_0 or SMART_DATA
           Component: Small Realtime Trace
           Description:
            Data 0: Source Address Register (ARC700)
            Data Register (ARC600)
         */
#define SMART_DATA_1         0x0702
        /* Symbol: SMART_DATA_1
           Component: Small Realtime Trace
           Description:
            Data 1: Destination Address Register (ARC700 only)
         */
#define SMART_DATA_2         0x0703
        /* Symbol: SMART_DATA_2
           Component: Small Realtime Trace
           Description:
            Data 2: Valid -Repeat-User/Kernel-ASID (ARC700 only)
         */
#define SMART_RESERVED_704   0x0704
#define SMART_RESERVED_7FF   0x07ff
        /* Symbol: SMART_<RESERVED_704,RESERVED_7FF>
           Component: Small Realtime Trace
           Description:
            Future SmaRT Developments
         */
#define AUX_XMAC1M           0x0803
#define AUX_XMAC2M           0x0801
#define AUX_XMAC1H           0x0802
#define AUX_XMAC2H           0x0803
        /* Symbol: AUX_XMAC<1M,2M,1H,2H>
           Component: dual 32x16 mul/mac
           Description:
            accumulator registers for dual32x16 mul/mac
            NOTE: AR600 only (reserved for ARC700)
         */
#define VRAPTOR_BASE         0x1000
#define VRAPTOR_LAST         0x1fff
        /* Symbol: VRAPTOR_<BASE,LAST>
           Component: VRaptor Platform
           Description:
            Dynamically Allocated Space for VRaptor Components
            (ARC700 only)
         */

/* Reserved: 0x2000 - 0x3fff XY Memory Mapping for debug access only */

#endif /* _AUX_DEFINES_NOT_WANTED */


#define REG_STATUS    (0x0) //   r  32  Basecase  Status Register
#define REG_STATUS_Z_BIT   (1 << 31)       // Zero flag
#define REG_STATUS_N_BIT   (1 << 30)       // Negative flag
#define REG_STATUS_C_BIT   (1 << 29)       // Carry flag
#define REG_STATUS_O_BIT   (1 << 28)       // Overflow flag
#define REG_STATUS_E2_BIT  (1 << 27)       // Enable Level 2 interrupt
#define REG_STATUS_E1_BIT  (1 << 26)       // Enable level 1 interrupt
#define REG_STATUS_H_BIT   (1 << 25)       // Halt
#define REG_STATUS_PC_MASK (0xFFFFFF)      // Obsolete A4 valid only

#define REG_SEMAPHORE (0x1) //   rw 4  Basecase  Inter-process/Host semaphore register
#define REG_LP_START  (0x2) //   rw  32  Basecase  Loop Start Address
#define REG_LP_END    (0x3) //   rw  32  Basecase  Loop End Address
#define REG_IDENTITY  (0x4) //   r  32  Basecase  ARC Identification Register
#ifdef _ATT4 // Just to prevent conversion with def2equ
#define REG_IDENTITY_CORE_VERSION  (_lr(IDENTITY) & 0xff)
#define REG_IDENTITY_CORE_NUMBER  (((_lr(IDENTITY) >> 8) & 0xff)
#define REG_IDENTITY_CHIP_ID  (((_lr(IDENTITY) >> 16) & 0xffff)
#endif

#define REG_DEBUG     (0x5) //   rw  32  Basecase  Debug Register
#define REG_DEBUG_LD_BIT   (1 << 31)  // Load pending bit
#define REG_DEBUG_SH_BIT   (1 << 30)  // Self halt bit (flag 1)
#define REG_DEBUG_BH_BIT   (1 << 29)  // Break halt bit
#define REG_DEBUG_UB_BIT   (1 << 28)  // User mode bit
#define REG_DEBUG_ED_BIT   (1 << 24)  // Enable debug (ARC600 and ARC700 only)
#define REG_DEBUG_SM_BITS  (7 << 24)  // Sleep Mode bits. (ARCv2 only)
#define REG_DEBUG_ZZ_BIT   (1 << 23)  // Sleeping bit
#define REG_DEBUG_RA_BIT   (1 << 22)  // Reset applied bit (written only by host)
#define REG_DEBUG_IS_BIT   (1 << 11)  // Instruction step bit
#define REG_DEBUG_ASR_BITS (0xff << 3)// Actionpoint status register bits
#define REG_DEBUG_AH_BIT   (1 << 2)   // Actionpoint halt flag
#define REG_DEBUG_FH_BIT   (1 << 1)   // Force Halt bit


#define REG_PC        (0x6) //   r  32  Basecase ARCompact Program Counter Register
// (0x7) reserved  n/a  n/a  reserved  reserved
// (0x8) reserved  n/a  n/a  reserved  reserved
// (0x9) reserved  n/a  n/a  reserved  reserved
#define REG_STATUS32    (0xA) // r  32  Basecase ARCompact Status Register
#define REG_STATUS32_L_BIT  (1 << 12)    // ZD Loop disabled
#define REG_STATUS32_Z_BIT  (1 << 11)    // Zero flag
#define REG_STATUS32_N_BIT  (1 << 10)    // Negative flag
#define REG_STATUS32_C_BIT  (1 << 9)     // Carry flag
#define REG_STATUS32_V_BIT  (1 << 8)     // Overflow flag
#define REG_STATUS32_U_BIT  (1 << 7)     // In User Mode ARC700
#define REG_STATUS32_DE_BIT (1 << 6)     // PC32 in delay slot ARC700
#define REG_STATUS32_AE_BIT (1 << 5)     // In Exception ARC700
#define REG_STATUS32_A1_BIT (1 << 4)     // In Level 2 Interrupt - ARC700
#define REG_STATUS32_A2_BIT (1 << 3)     // In Level 1 Interrupt - ARC700
#define REG_STATUS32_E2_BIT (1 << 2)     // Enable Level 2 interrupt
#define REG_STATUS32_E1_BIT (1 << 1)     // Enable level 1 interrupt
#define REG_STATUS32_H_BIT (1)           // Halt
#define REG_STATUS32_L1 (0xB) // rw  32  Basecase ARCompact Status Register save for level 1
#define REG_STATUS32_L1_L_BIT  (1 << 12) // ZD Loop disabled
#define REG_STATUS32_L1_Z_BIT  (1 << 11) // Zero flag
#define REG_STATUS32_L1_N_BIT  (1 << 10) // Negative flag
#define REG_STATUS32_L1_C_BIT  (1 << 9)  // Carry flag
#define REG_STATUS32_L1_V_BIT  (1 << 8)  // Overflow flag
#define REG_STATUS32_L1_U_BIT  (1 << 7)  // In User Mode ARC700
#define REG_STATUS32_L1_DE_BIT (1 << 6)  // PC32 in delay slot ARC700
#define REG_STATUS32_L1_AE_BIT (1 << 5)  // In Exception ARC700
#define REG_STATUS32_L1_A1_BIT (1 << 4)  // In Level 2 Interrupt - ARC700
#define REG_STATUS32_L1_A2_BIT (1 << 3)  // In Level 1 Interrupt - ARC700
#define REG_STATUS32_L1_E2_BIT (1 << 2)  // Enable Level 2 interrupt
#define REG_STATUS32_L1_E1_BIT (1 << 1)  // Enable level 1 interrupt

#define REG_STATUS32_L2 (0xC) // rw  32  Basecase ARCompact Status Register save for level 2
#define REG_STATUS32_L2_L_BIT  (1 << 12) // ZD Loop disabled
#define REG_STATUS32_L2_Z_BIT  (1 << 11) // Zero flag
#define REG_STATUS32_L2_N_BIT  (1 << 10) // Negative flag
#define REG_STATUS32_L2_C_BIT  (1 << 9)  // Carry flag
#define REG_STATUS32_L2_V_BIT  (1 << 8)  // Overflow flag
#define REG_STATUS32_L2_U_BIT  (1 << 7)  // In User Mode ARC700
#define REG_STATUS32_L2_DE_BIT (1 << 6)  // PC32 in delay slot ARC700
#define REG_STATUS32_L2_AE_BIT (1 << 5)  // In Exception ARC700
#define REG_STATUS32_L2_A1_BIT (1 << 4)  // In Level 2 Interrupt - ARC700
#define REG_STATUS32_L2_A2_BIT (1 << 3)  // In Level 1 Interrupt - ARC700
#define REG_STATUS32_L2_E2_BIT (1 << 2)  // Enable Level 2 interrupt
#define REG_STATUS32_L2_E1_BIT (1 << 1)  // Enable level 1 interrupt

// (0xD) reserved

#define REG_IC_IVIC (0x10)    // w  0  ICACHE  invalidate instruction cache CHE_MODE  rw  5

#define REG_IC_CTRL (0x11)    // rw  6  ICACHE cache control
#define REG_IC_CTRL_DC_BIT (1)  // DC[0]  Disable Cache: - Enables/Disables the cache ( 0 = Enable Cache , 1 = Disable Cache ) R/W
#define REG_IC_CTRL_EB_BIT  (1 << 1)  //  EB[1]  Bypass: - For ARC 700 the bypass is always enabled 1 =  Bypass Enabled  R (ARC700) R/W earlier cores
// Reserved [2]
#define REG_IC_CTRL_SB_BIT  (1 << 3)  //   SB[3]  Success Bit: - Success of last cache operation (0 = failed, 1=success) R/W
#define REG_IC_CTRL_RA_BIT  (1 << 4)  //   RA[4]  Replacement Algorithm: - ARC 700 uses random replacement only (0 = Random Replacement, 1= round robin) ARC700 R, earlier cores R/W
#define REG_IC_CTRL_AT_BIT  (1 << 5)   //   AT[5]  Address Debug Type: - Used for debug purposes for when accessing cache RAMs (0 = Direct Cache RAM Access , 1 = Cache Controlled RAM Access ) R/W
// Reserved[31:6] -- --


#define REG_MULHI   (0x12)    // w  32  MUL   High part of multiply to restore multiply state
#define REG_IC_LIL  (0x13)    // w  32  MULTI-WAY ICACHE cache lock instruction line
#define REG_DMC_CODE_RAM  (0x14) // rw  25  CACHE CONTROLLER start address of DMC code RAM
#define REG_TAG_ADDR_MASK  (0x15)   // 0x15  TAG_ADDR_MASK  Direct Mapped Instruction Cache  (Virtual Cache) Mask for tag address
#define REG_TAG_DATA_MASK    (0x16)   //  Direct Mapped Instruction Cache  (Virtual Cache) Mask for tag data
#define REG_LINE_LENGTH_MASK    (0x17)   // Direct Mapped Instruction Cache  Mask for line length counter
#define REG_AUX_LDST_RAM  (0x18) // rw  32  LOAD STORE RAM  Address of local load store RAM
#define REG_IC_IVIL (0x19)   //  w  32   ICACHE : cache invalidate instruction line (unlocks as well
#define REG_IC_RAM_ADDRESS (0x1A)  // rw  32  MULTI-WAY ICACHE :  cache ram access address
#define REG_IC_TAG  (0x1B)  // rw  32  MULTI-WAY ICACHE tag ram access
#define REG_IC_WP   (0x1C) // rw  32  MULTI-WAY ICACHE Way pointer access
#define REG_IC_DATA (0x1D) //  rw  32  MULTI-WAY ICACHE data ram access
// (0x1E)   reserved
// (0x1F)   reserved  n/a  n/a  reserved  reserved
#define REG_SRAM_SEQ (0x20) // rw  8  SRAM SEQUENCER SRAM sequencer mode control
#define REG_COUNT    (0x21) // rw  32  Enhanced Timer 0   Timer 0 Count value
#define REG_CONTROL  (0x22) // rw  3  Enhanced Timer 0  Timer 0 Control value
#define REG_CONTROL_IE_BIT (1) // rw  Interrupt Enable
#define REG_CONTROL_NH_BIT (1<<1) // rw  Not handled bit - timer does not tick in debug halt
#define REG_CONTROL_W_BIT (1<< 2) // rw   Watch dog bit (reset on timer wrap)
#define REG_CONTROL_IP_BIT (1 << 3) // r Interrupt Pending bit
#define REG_LIMIT    (0x23) // rw  32  Enhanced Timer 0  Timer 0 Limit value
#define REG_COUNT0    (0x21) // rw  32  Enhanced Timer 0   Timer 0 Count value
#define REG_CONTROL0  (0x22) // rw  3  Enhanced Timer 0  Timer 0 Control value
#define REG_CONTROL0_IE_BIT (1) // rw  Interrupt Enable
#define REG_CONTROL0_NH_BIT (1<<1) // rw  Not handled bit - timer does not tick in debug halt
#define REG_CONTROL0_W_BIT (1<< 2) // rw   Watch dog bit (reset on timer wrap)
#define REG_CONTROL0_IP_BIT (1 << 3) // r Interrupt Pending bit
#define REG_LIMIT0    (0x23) // rw  32  Enhanced Timer 0  Timer 0 Limit value

#ifdef _ATT4 // Just to prevent conversion with def2equ
// Some timer0 macros
#define START_TIMER()   _sr(2,REG_CONTROL0); _sr(-1,REG_LIMIT0); _sr(0,REG_COUNT0)
#define START_TIMER0()   _sr(2,REG_CONTROL0); _sr(-1,REG_LIMIT0); _sr(0,REG_COUNT0)
#define GET_TIMER()   _lr(REG_COUNT0)
#define GET_TIMER0()   _lr(REG_COUNT0)
#endif  // prevent conversion with def2equ



#define REG_PCPORT   (0x24) // w  1  PC PORT  Control for PC_SEL line
#define REG_INT_VECTOR_BASE   (0x25) // rw 32  Basecase ARCompact Interrupt Vector Base address
#define REG_AUX_VBFDW_MODE    (0x26) // rw  16  Viterbi Instruction  VBFDW Instruction Mode
#define REG_AUX_VBFDW_BM0     (0x27) // rw  16/16  Viterbi Instruction  Branch Metric 0
#define REG_AUX_VBFDW_BM1     (0x28) // rw  16/16  Viterbi Instruction  Branch Metric 1
#define REG_AUX_VBFDW_ACCU    (0x29) // rw  32  Viterbi Instruction  VBFDW Accumulator
#define REG_AUX_VBFDW_OFST    (0x2A) // rw  13  Viterbi Instruction  XY Memory path metric offset
#define REG_AUX_VBFDW_INTSTAT (0x2B) // rw  11  Viterbi Instruction  VBFDW Internal State
#define REG_AUX_XMAC0_24      (0x2C) // rw  32  XMAC DSP V3.1   Accumulator XTP
#define REG_AUX_XMAC1_24      (0x2D) // rw  32  XMAC DSP V3.1   Accumulator MSP
#define REG_AUX_XMAC2_24      (0x2E) // rw  32  XMAC DSP V3.1   Accumulator LSP
#define REG_AUX_FBF_STORE_16  (0x2F) // rw  30  XMAC FBF DSP V3.1  FBF OPERAND STORAGE
// (0x30) reserved
// (0x31) reserved
#define REG_AUX_CRC_POLY (0x32) // r/w  32  PROGRAMMABLE  CRC POLYNOMIAL REGISTER
#define REG_AUX_CRC_MODE (0x33) // r/w  1  PROGRAMMABLE     CRC   MODE REGISTER
// 0x34 - 0x3F reserved
#define REG_XTP_NEWVAL   (0x40) // r/w  4  MAC  The 4-bit extended product of the multiply-accumulate result
#define REG_AUX_MACMODE  (0x41) // r/w 32  XMAC DSP v2.1/DSP v3.0

#define REG_LSP_NEWVAL   (0x42) // r/w  32  MAC  Low 32 bit register product
#define REG_AUX_IRQ_LV12 (0x43) // rw  2  Interrupt subsystem  Sticky flags for interrupts levels 1 and 2
#define REG_AUX_XMAC0    (0x44) // rw  0  XMAC DSP v3.0  Accumulator Extended Product
#define REG_AUX_XMAC1    (0x45) // rw  0  XMAC DSP v3.0  Accumulator Most Significant Part
#define REG_AUX_XMAC2    (0x46) // rw  0  XMAC DSP v3.0  Accumulator Least Significant Part
#define REG_DC_IVDC      (0x47) // w  0  DATA CACHE  Invalidate Data Cache

#define REG_DC_CTRL      (0x48) // rw  8  DATA CACHE  Control register
#define REG_DC_CTRL_DC_BIT (1)  // DC[0]  Disable Cache: - Enables/Disables the cache ( 0 = Enable Cache , 1 = Disable Cache ) R/W
#define REG_DC_CTRL_EB_BIT  (1 << 1) //  EB[1]  Bypass: - For ARC 700 the bypass is always enabled 1 =  Bypass Enabled  R (ARC700) R/W earlier cores
#define REG_DC_CTRL_SB_BIT  (1 << 2) //  SB[2]  Success Bit: - Success of last cache operation (0 = failed, 1=success) R/W
#define REG_DC_CTRL_RA_BIT  (3 << 3) //  RA[4]  Replacement Algorithm: - ARC 700 uses random replacement only (0 = Random Replacement, 1= round robin) ARC700 R, earlier cores R/W
#define REG_DC_CTRL_AT_BIT  (1 << 5) //   AT[5]  Address Debug Type: - Used for debug purposes when accessing cache RAMs (0 = Direct Cache RAM Access , 1 = Cache Controlled RAM Access ) R/W
#define REG_DC_CTRL_IM_BIT  (1 << 6) //  IM[6]  Invalidate Mode: - Selects the invalidate type ( 0 = invalidate only, 1= invalidate and flush) R/W
#define REG_DC_CTRL_M_BIT  (1 << 7)  //  LM[7] Lock Mode: - Selects the effect of a flush command on a locked entry (0=enable flush, 1 = disable flush) R/W
#define REG_DC_CTRL_FS_BIT  (1 << 8) //  FS[8]  Flush Status:- Status of the data cache flush mechanism (0 = idle, 1 = flush in progress) R
// Reserved [31:9]

#define REG_DC_LDL       (0x49) // w  32  DATA CACHE  Lock data line
#define REG_DC_IVDL      (0x4A) // w  32  DATA CACHE  Invalidate data line
#define REG_DC_FLSH      (0x4B) // w  0  DATA CACHE  Flush data cache
#define REG_DC_FLDL      (0x4C) // w  32  DATA CACHE  Flush data line
//  0x4D - 0x52 reserved
#define REG_AUX_LCD_DATA  (0x53) //  w  8  ARCangel 3 LCD  LCD display write data
#define REG_AUX_LCD_CNTRL (0x54) // w  32  ARCangel 3 LCD  LCD display control register final
#define REG_AUX_US_COUNT  (0x55) // w  16  ARCangel 3 LCD  LCD working frequency
//  (0x56) reserved
#define REG_AUX_LCD_BUTTONISTER (0x57) //  rw  32  ARCangel 3  Front panel button auxiliary register
#define REG_DC_RAM_ADDR (0x58) //  rw  32  DATA CACHE  Address for data cache RAM access
#define REG_DC_TAG  (0x59) // rw  32  DATA CACHE   Tag data associated with dc_ram_addr
#define REG_DC_WP   (0x5A) // rw  32  DATA CACHE  Way Pointer value associated with dc_ram_addr
#define REG_DC_DATA (0x5B) //  rw  32  DATA CACHE   Data longword associated with dc_ram_addr
//  0x5C reserved  n/a n/a  reserved  reserved
//  0x5D reserved  n/a n/a  reserved  reserved
//  0x5E reserved  n/a n/a  reserved  reserved
//  0x5F reserved  n/a n/a  reserved  reserved
//  0x60-0x7F BCR Area Refer to BCR Section Build configuration
// 0x60  Reserved
#define REG_DCCM_BASE_BUILD (0x61) //  r  Base Address Data Closely Coupled Memory (DCCM)
#define REG_CRC_BUILD_BCR     (0x62) //  r  CRC instruction
#define REG_BTA_LINK_BUILD    (0x63) //  r  BTA_L1/BTA_L2 registers
#define REG_RTT_BUILD         (0x63) //  r  Real-Time Trace Unit
#define REG_DVBF_BUILD        (0x64) //  r  Dual viterbi butterfly instruction
#define REG_TEL_INSTR_BUILD   (0x65) //  r  DSP v3.0 instructions package
#define REG_DATASPACE         (0x66) //  r  Dataspace
#define REG_MEMSUBSYS         (0x67) //  r  Memory subsystem
#define REG_VECBASE_AC_BUILD  (0x68) //  r  ARCompact Interrupt Vector Base Address
#define REG_P_BASE_ADDR       (0x69) //  r  Peripheral base address
#define REG_DATA_UNCACHED_BUILD  (0x6A) //   Data Uncached Build Configuration Register
//  0x6B-0x6D Reserved
#define REG_RF_BUILD          (0x6E) // RF_BUILD         r  Core Registers
#define REG_MMU_BUILD         (0x6F) // MMU_BUILD        r  Memory Management Unit (MMU)
//   0x70-0x71  Reserved
#define REG_D_CACHE_BUILD   (0x72)  //   r  Data Cache
#define REG_MADI_BUILD      (0x73)  //   r  Multiple ARC Debug Interface
#define REG_LDSTRAM_BUILD   (0x74)  //   r  LD/ST RAM
#define REG_DCCM_BUILD      (0x74)  //   r  Data Closely Coupled Memory (ARC 700)
#define REG_TIMER_BUILD     (0x75)  //   r  Timer
#define REG_AP_BUILD        (0x76)  //   r  Actionpoints
#define REG_I_CACHE_BUILD   (0x77)  //   r  Instruction Cache
#define REG_ADDSUB_BUILD    (0x78)  //   r  Saturated Add/Sub (obsolete)
#define REG_ICCM_BUILD      (0x78)  //   r  Instruction Closely Coupled Memory (ARC 700)
#define REG_DSPRAM_BUILD    (0x79)  //   r  Scratchpad and XY Memory Build Configuration Register (DSPRAM_BUILD)
// The BCR register for XY memory is a read only register used to allow the debug host and kernel
// mode processes to obtain information about the configuration of the XY memory module. It has three
// fields that describe the version of the memory and it.s memory regions size, and number of banks.
// These are defined during build time. The version number of the XY memory for ARC 700 processor
// is currently set to 4. This register is available to kernel mode processes when an OS is present.  User
// mode access to the register is controlled by the XPU register (see Extension Permission ).

#ifdef _ATT4 // Just to prevent conversion with def2equ
#define REG_DSPRAM_BUILD_VERSION (_lr(DSPRAM_BUILD ) & 0xFF)  //  [7:0]  Current version 0x04
#define REG_DSPRAM_BUILD_SIZE (1024 << (((_lr(DSPRAM_BUILD ) >> 8 ) & 0xF) -1))  // [11:8]  Bank RAM Size (for both X and Y)
#define REG_DSPRAM_BUILD_BANKS ((_lr(DSPRAM_BUILD ) >> 12 ) & 0xF))  // [11:8]  Bank RAM Size (for both X and Y)
#endif
#define REG_MAC_BUILD       (0x7A)  //   r  MUL\MAC
#define REG_MULTIPLY_BUILD  (0x7B)  //   r  Multiply
#define REG_SWAP_BUILD      (0x7C)  //   r  Swap
#define REG_NORM_BUILD      (0x7D)  //   r  Normalize
#define REG_MINMAX_BUILD    (0x7E)  //   r  MIN\MAX
#define REG_BARREL_BUILD    (0x7F)  //   r  Barrel shift
#define REG_AX0   (0x80) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AX1   (0x81) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AX2   (0x82) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AX3   (0x83) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AY0   (0x84) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AY1   (0x85) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AY2   (0x86) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_AY3   (0x87) // rw  (7-13)  XY MEMORY DSP v3.0 Address register
#define REG_MX00  (0x88) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX01  (0x89) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX10  (0x8A) //  rw  32  XY MEMORY DSP  Modifier register
#define REG_MX11  (0x8B) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX20  (0x8C) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX21  (0x8D) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX30  (0x8E) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MX31  (0x8F) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY00  (0x90) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY01  (0x91) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY10  (0x92) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY11  (0x93) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY20  (0x94) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY21  (0x95) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY30  (0x96) //  rw  32  XY MEMORY DSP v3.0 Modifier register
#define REG_MY31  (0x97) //  rw  32  XY MEMORY DSP v3.0 Modifier register

// 31:30  AM  This defines the addressing mode. Options available:
// 00]: AM_MODULO Modulo addressing is selected when this constant
// is used. In this mode, bits 16 to 28 are used to define the modulo range.
// The address offset is specified using bits 0 to 13. This mode is also used
// for linear addressing, which is enabled by setting the modulo field to
// zero.

#define REG_XY_MODIFIER_MODULO (1)

// [01]: AM_BITREVERSE Reverse-carry addressing is selected when
// this constant is used. The address offset should be unsigned when this addressing mode is selected.

#define REG_XY_MODIFIER_BIT_REVERSE (1)

// [10,11]: These address modes are reserved for future expansion.

// 29  H  Half word mode.
// When set to one, each location is 16 bits. While in this mode:
// For a read operation, the 16-bit data read is always placed into the top
// half of a 32-bit word with the lower 16 bits cleared to zeros.
// For write operation, only the top 16 bits of the 32-bit word are used for
// writing. When set to zero, each location is 32 bits.

#define REG_XY_MODIFIER_32BIT_MODE (0)
#define REG_XY_MODIFIER_16BIT_MODE (1)


// 28:16  MODULO  This defines the buffer range for modulo addressing. The value should
// always be specified as an unsigned number, and must not be set to a
// number bigger than the number of locations in a memory region. When
// this field is cleared to all zeros, the complete address range of the
// selected memory region is made available (linear addressing).

#define REG_XY_MODIFIER_MODULO_LINEAR_SIZE (0)


// 15  D  Dual addressing mode. The dual mode is used with the 16-bit
// addressing mode. When enabled, the same 16-bit data is returned in
// both the lower and upper half of the 32-bit data word.
// This mode minimizes data memory use when the same data must be
// applied to both channels.

#define REG_XY_MODIFIER_DUAL_ADDRESS_ON (1)
#define REG_XY_MODIFIER_DUAL_ADDRESS_OFF (0)

//14  Reserved  Reserved for future expansion.  Writing to this field will have to effect
// and zero will be returned upon reading.  The programmer should not
// use or rely on this behavior

// 13:0  N  Bits 0 to 13 define the address-offset (stride) value. The value contained
// in this field is added to the respective address register value when its update pointer is used.
//  The offset value is specified as a 2s complement number.

#ifdef _ATT4 // Just to prevent conversion with def2equ

#define SET_REG_XY_MODIFIER(address_mode,half_word_mode,modulo_size, dual_address_mode,address_offset, modifier_reg) \
                            _sr(((address_mode) << 30 | (half_word_mode) << 29 |  (modulo_size) << 16 |  (dual_address_mode) << 15 | \
                                                              (address_offset)), modifier_reg)

#endif


#define REG_XYCONFIG  (0x98) //  rw  32  XY MEMORY DSP v3.0 Configuration Register
#define REG_XYCONFIG_ER_BIT (1 << 16)  //  ER  R  Burst Error Bit. This set to high if a bus error is encountered during a burst transfer. This bit is cleared whenever a new burst unit transfer is started by writing to BURSTSZ.
// 15:14  Reserved  R  Reserved for future expansion.  Writing to this field will have no effect and zero will be returned upon reading.  The programmer should not use or rely on this behavior
#define REG_XYCONFIG_FY_BIT (1 << 13)  //    FY  W  Flush Y region. When writing one to this field, the burst unit will flush the complete Y region to system memory, using XYLSBASEY as the system memory base address. This bit always returns zeros when read.
#define REG_XYCONFIG_FX_BIT (1 <<  12)  //   FX  W  Flush X region. When writing one to this field, the burst unit will flush the complete X region to system memory, using XYLSBASEX as the system memory base address. This bit always returns zeros when read.
#define REG_XYCONFIG_PY_BIT (1 << 11)  //   PY  W  Pre-load Y region. When writing one to this field, the burst unit will pre-load the complete Y region from system memory, using XYLSBASEY as the system memory base address. This bit always returns zeros when read.
#define REG_XYCONFIG_PX_BIT (1 <<  10)  //    PX  W  Pre-load X region. When writing one to this field, the burst unit will pre-load the complete X region from system memory, using XYLSBASEX as the system memory base address. This bit always returns zeros when read.
#define REG_XYCONFIG_BI_BIT (1 <<  9)  //    BI  R/W  Burst Interrupt enable bit. When set, XY module will produce a level 1 interrupt on completion of a burst operation. #define REG_XYCONFIG_LO_BIT (1 <<  8)  //    LO  R/W  Lock out bit. When set, will stall the processor if an attempt is made to access a memory region of a bank while it is being used in a burst operation.
#define REG_XYCONFIG_DU_BIT (1 <<  7)  //    DU  R  Bank used bit. When .1., indicates that current bank selected using BS is being used in a DMI operation. When DMI port does not exist, this bit is reserved, and writing to this field will have no effect and zero will be returned upon reading.
#define REG_XYCONFIG_DP_BIT (1 <<  6)  //    DP  R  DMI in progress bit. When 1, indicates that a DMI operation is in progress. When DMI port does not exist, this bit is reserved, and writing to this field will have no effect and zero will be returned upon reading.
#define REG_XYCONFIG_BU_BIT (1 <<  5)  //    BU  R  Bank used bit. When 1, indicates that current bank selected using BS is being used in a burst operation.
#define REG_XYCONFIG_BP_BIT (1 <<  4)  //    BP  R  Burst in progress bit. When 1, indicates that a burst operation is in progress.
// 3:2  Reserved  R  Reserved for future expansion.  Writing to this field will have no effect and zero will be returned upon reading.
#define REG_XYCONFIG_BS_MASK (3)  //  (0:1) BS  R  Bank select bits. Used to select which bank should be switched in for pointer access operation. Currently this value is hard set to .00. and is read only.

#ifdef _ATT4 // Just to prevent conversion with def2equ
#define SET_XY_BANK(x) _sr(x, REG_XYCONFIG)
#endif

// bursting
#define REG_BURSTSYS  (0x99) //  rw  32  XY MEMORY DSP v3.0 Starting burst system address
#define REG_BURSTVAL  (0x9C) //  rw  32  XY MEMORY DSP v3.0 Burst fill value register
#define REG_BURSTXYM  (0x9A) //  r/w 32  Starting burst XY memory address

#define REG_BURSTSZ  (0x9B) //  r/w 32  Burst size and destination register
//  BIT 31  Reserved  Reserved for future expansion.  Writing to this field will have no effect and zero will be returned upon reading.
//  The programmer should not use or rely on this behavior.
#define REG_BURSTSZ_BD_BIT  (1 << 30) //  BD  Burst Direction bit.
// For normal burst operations, if BD is set, data is transferred from main memory to XY memory, else data is transferred from XY memory to main memory.
// For fill operations, if BD is set, the target is XY memory, else the target is main memory

#define BURST_TO_XY (1)
#define BURST_TO_SYS (0)



#define REG_BURSTSZ_RS_BIT  (1 <<   29 ) //  RS  Region Select bit, RS, select which region of the XY memory is targeted for transfer.
//   When cleared, X memory is targeted, else Y memory is targeted.

#define BURST_REGION_X (0)
#define BURST_REGION_Y (1)

#define REG_BURSTSZ_F_BIT  (1 <<   28)  //  F  Burst fill bit. When set during a write to this register, a fill operation is performed using a constant value defined by BURSTVAL.
//  BIT 27:26  Reserved  Reserved for future expansion.  Writing to this field will have to effect and zero will be returned upon reading.  The programmer should not use or rely on this behavior.
#define REG_BURSTSZ_BS_MASK  (3 <<  24) //   BS  Bank Select. Defines the bank (if required) to be used in the burst operation.
#define BURST_BANK_1 (0)
#define BURST_BANK_2 (1)
#define BURST_BANK_3 (2)
#define BURST_BANK_4 (3)
#define REG_BURSTSZ_BURSTSIZE_MASK  (0xFF) ///  BURSTSIZE  The size of the burst unit transfer in bytes minus 1, and the burst size must be specified in 4 byte (32bits) chunks.
//  The number of bits of this field length is dependant upon the size


#ifdef _ATT4 // Just to prevent conversion with def2equ

// NOTE size for bursting must be total bytes -1   Leave bank as zero for one bank systems, xy is 0=X and 1=Y region

#define BURST_MEM_XY BURST_SYS_TO_XY    // not clear macro name

#define BURST_XY_MEM  BURST_XY_TO_SYS   // not clear macro name

#define BURST_SYS_TO_XY(bank,xy,dest,src,size_in_bytes_minus_one) _sr((unsigned long)src,REG_BURSTSYS); \
    _sr(dest,REG_BURSTXYM); \
    _sr((size_in_bytes_minus_one | (bank<<24) | (xy<<29) | REG_BURSTSZ_BD_BIT  ), REG_BURSTSZ  )

#define BURST_XY_TO_SYS(dest,bank,xy,src,size_in_bytes_minus_one) _sr((unsigned long)dest,REG_BURSTSYS); \
    _sr(src,REG_BURSTXYM); \
    _sr((size_in_bytes_minus_one | (bank<<24) | (xy<<29)),REG_BURSTSZ  )

#define BURST_FILL_XY(bank,xy,dest,pat,size_in_bytes_minus_one) _sr(pat,REG_BURSTVAL); \
    _sr(dest,REG_BURSTXYM); \
    _sr((size_in_bytes_minus_one | (bank<<24) | (xy<<29) | REG_BURSTSZ_BD_BIT   | REG_BURSTSZ_F_BIT ),REG_BURSTSZ  )

#define BURST_FILL_MEM(dest,pat,size_in_bytes_minus_one) _sr(pat,REG_BURSTVAL); \
    _sr((int)dest,REG_BURSTSYS); \
    _sr((size_in_bytes_minus_one | REG_BURSTSZ_F_BIT ),REG_BURSTSZ  )

#define BURST_WAIT() while((_lr(REG_XYCONFIG) & REG_XYCONFIG_BP_BIT ) != 0)

#endif  // prevent conversion with def2equ


#define REG_XYLSBASEX (0x9D) //  r/w 24-32  Load store base address for XY memory (current X memory) Default width is 24 bit.
// For more information see Memory Components Reference
#define REG_XYLSBASEY (0x9E) //  r/w 24-32  Load store base address for XY memory (current Y memory).
// Default width is 24 bit. For more information see Memory Components Reference
// 0x9F - 0xFF reserved n/a  n/a  reserved  reserved
#define REG_COUNT1  (0x100)  // rw  32  Enhanced Timer 1  Timer 1 Count value
#define REG_CONTROL1 (0x101) // rw  3  Enhanced Timer 1  Timer 1 Control value
#define REG_CONTROL1_IE_BIT (1) // rw  Interrupt Enable
#define REG_CONTROL1_NH_BIT (1<<1) // rw  Not handled bit - timer does not tick in debug halt
#define REG_CONTROL1_W_BIT (1 << 2) // rw   Watch dog bit (reset on timer wrap)
#define REG_CONTROL1_IP_BIT (1 << 3) // r Interrupt Pending bit
#define REG_LIMIT1  (0x102)  // rw  32  Enhanced Timer 1  Timer 1 Limit value


#ifdef _ATT4 // Just to prevent conversion with def2equ
// Some timer1 macros
#define START_TIMER1()   _sr(2,REG_CONTROL1); _sr(-1,REG_LIMIT1); _sr(0,REG_COUNT1)
#define GET_TIMER1()   _lr(REG_COUNT1)
#endif  // prevent conversion with def2equ


// 0x103 - 0x11F reserved for future timers n/a  n/a
// reserved  ARC long Immediate auxiliary address allocation
// 0x120 - 0x13F reserved for ARCangel peripherals n/a  n/a  reserved  ARC long Immediate auxiliary address allocation
// 0x140 - 0x1FF reserved for peripherals n/a  n/a  reserved  ARC long Immediate auxiliary address allocation
#define REG_AUX_IRQ_LEV  (0x200) // rw  29  Interrupt subsystem  Level1, Level2 programming register
#define REG_AUX_IRQ_HINT (0x201) // w  29  Interrupt subsystem  Software interrupt trigger location
//  0x202 - 0x21F not allocated  n/a  n/a  ARC  ARC long Immediate auxiliary address allocation

#define REG_USTACK_TOP   (0x260)
#define USTACK_TOP       ((void*)REG_USTACK_TOP)
#define REG_USTACK_BASE  (0x261)
#define USTACK_BASE      ((void*)REG_USTACK_BASE)
#define REG_KSTACK_TOP   (0x264)
#define KSTACK_TOP       ((void*)REG_KSTACK_TOP)
#define REG_KSTACK_BASE  (0x265)
#define KSTACK_BASE      ((void*)REG_KSTACK_BASE)

#define REG_AP_AMV0 (0x220)  //  rw  32  ACTIONPOINTS  Actionpoint 0 Match Value
#define REG_AP_AMM0 (0x221)  //  w  32  ACTIONPOINTS  Actionpoint 0 Match Mask
#define REG_AP_AC0  (0x222)  //  w  32  ACTIONPOINTS  Actionpoint 0 Control
#define REG_AP_AMV1 (0x223)  //  rw  32  ACTIONPOINTS  Actionpoint 1 Match Value
#define REG_AP_AMM1 (0x224)  //  w  32  ACTIONPOINTS  Actionpoint 1 Match Mask
#define REG_AP_AC1  (0x225)  //  w  32  ACTIONPOINTS  Actionpoint 1 Control
#define REG_AP_AMV2 (0x226)  //  rw  32  ACTIONPOINTS  Actionpoint 2 Match Value
#define REG_AP_AMM2 (0x227)  //  w  32  ACTIONPOINTS  Actionpoint 2 Match Mask
#define REG_AP_AC2  (0x228)  //  w  32  ACTIONPOINTS  Actionpoint 2 Control
#define REG_AP_AMV3 (0x229)  //  rw  32  ACTIONPOINTS  Actionpoint 3 Match Value
#define REG_AP_AMM3 (0x22a)  //  w  32  ACTIONPOINTS  Actionpoint 3 Match Mask
#define REG_AP_AC3  (0x22b)  //  w  32  ACTIONPOINTS  Actionpoint 3 Control
#define REG_AP_AMV4 (0x22c)  //  rw  32  ACTIONPOINTS  Actionpoint 4 Match Value
#define REG_AP_AMM4 (0x22d)  //  w  32  ACTIONPOINTS  Actionpoint 4 Match Mask
#define REG_AP_AC4  (0x22e)  //  w  32  ACTIONPOINTS  Actionpoint 4 Control
#define REG_AP_AMV5 (0x22f)  //  rw  32  ACTIONPOINTS  Actionpoint 5 Match Value
#define REG_AP_AMM5 (0x230)  //  w  32  ACTIONPOINTS  Actionpoint 5 Match Mask
#define REG_AP_AC5  (0x231)  //  w  32  ACTIONPOINTS  Actionpoint 5 Control
#define REG_AP_AMV6 (0x232)  //  rw  32  ACTIONPOINTS  Actionpoint 6 Match Value
#define REG_AP_AMM6 (0x233)  //  w  32  ACTIONPOINTS  Actionpoint 6 Match Mask
#define REG_AP_AC6  (0x234)  //  w  32  ACTIONPOINTS  Actionpoint 6 Control
#define REG_AP_AMV7 (0x235)  //  rw  32  ACTIONPOINTS  Actionpoint 7 Match Value
#define REG_AP_AMM7 (0x236)  //  w  32  ACTIONPOINTS  Actionpoint 7 Match Mask
#define REG_AP_AC7  (0x237)  //  w  32  ACTIONPOINTS  Actionpoint 7 Control
// 0x238 - 0x2FF not allocated  n/a  n/a  ARC  ARC long Immediate auxiliary address allocation
// 0x300 - 0x3FF reserved n/a  n/a  ARC  reserved

#define REG_ERET (0x400)       //      r/w  Exception Return Address
#define REG_ERBTA  (0x401)       //     r/w  Exception Return Branch Target Address
#define REG_ERSTATUS  (0x402)     // r/w  Exception Return Status
#define REG_ECR     (0x403)       //   r  Exception Cause Register
#define REG_EFA     (0x404)       //     r/w  Exception Fault Address
#define REG_ICAUSE1   (0x40A)       //     r  Level 1 Interrupt Cause Register
#define REG_ICAUSE2  (0x40B)       //    r  Level 2 Interrupt Cause Register
#define REG_AUX_IENABLE  (0x40C)       //     r/w  Interrupt Mask Programming
#define REG_AUX_ITRIGGER (0x40D)       //     r/w  Interrupt Sensitivity Programming
#define REG_XPU  (0x410)       //       r/w  User Mode Extension Enables
#define REG_XPK  (0x411)       //     r/w  User Mode Extension Enables
#define REG_BTA  (0x412)       //      Branch Target Address
#define REG_BTA_L1 (0x413)       //     r/w  Level 1 Return Branch Target
#define REG_BTA_L2 (0x414)       //    r/w  Level 2 Return Branch Target
#define REG_AUX_IRQ_PULSE_CANCEL  (0x415)       //    w  Interrupt Pulse Cancel
#define REG_AUX_IRQ_PENDING  (0x416)       //     r  Interrupt Pending Register

// 0x417 - 0x1FFF not allocated  n/a  n/a  ARC  ARC long Immediate auxiliary address allocation Build Configuration Registers

#define MPU_ECR   ((void*)0x420)  /* MPU exception cause register. */


// 0x2000 - 0x3FFF X_MEMORY  rw  32  XY MEMORY  X Memory Mapping for debug access only
// 0x2000 - 0x3FFF Y_MEMORY  rw  32  XY MEMORY  Y Memory Mapping for debug access only
// 0x4000 - 0x7FFFFFFF not allocated  n/a  n/a  ARC  ARC long Immediate auxiliary address allocation
// 0x80000000 - 0xFFFFF7FF available to customer n/a  n/a  Customer  Customer long Immediate auxiliary address allocation
// 0xFFFFF800 - 0xFFFFFFFF available to customer       Customer short range auxiliary address allocation

// Interrupt and Exception Vectors - including ARC700

#define EV_RESET (0)   //   Exception Vector Reset -
#define EV_RESET_ADDRESS (0)   //   Exception Vector Reset -
#define EV_RESET_NUM (0)   //   Exception Vector Reset -
#define EV_MEM_ERR (8)  //   Exception Vector Memory Error
#define EV_MEM_ERR_ADDRESS (8)  //   Exception Vector Memory Error
#define EV_MEM_ERR_NUM (1)  //   Exception Vector Memory Error
#define EV_INSTR_ERR (0x10)  //   Exception Vector Instruction Error
#define EV_INSTR_ERR_ADDRESS (0x10)  //   Exception Vector Instruction Error
#define EV_INSTR_ERR_NUM (2)  //   Exception Vector Instruction Error
#define EV_INSTR_ERR_ILLEGAL_CODE (0)  //   Exception Vector Instruction Error - Illegal Instruction
#define EV_INSTR_ERR_ILLEGAL_SEQ_CODE (1)  //   Exception Vector Instruction Error - Illegal Instruction Sequence
#define EV_INSTR_ERR_ECR (0x020000)  //   Exception Vector Instruction Error - Exception Cause Register


#define IV_IRQ3 (3*8)  //  Interrupt vector 3 (Timer 0) - default level 1- default priority Low27 - ARC700 Low01
#define IV_IRQ3_ADDRESS (3*8)  //  Interrupt vector 3 (Timer 0) - default level 1- default priority Low27 - ARC700 Low01
#define IV_IRQ3_NUM (3)  //  Interrupt vector 3 (Timer 0) - default level 1- default priority Low27 - ARC700 Low01

#define IV_TIMER0 IV_IRQ3
#define IV_TIMER0_ADDRESS IV_IRQ3_ADDRESS
#define IV_TIMER0_NUM IV_IRQ3_NUM

#define IV_IRQ4 (4*8)  //  Interrupt vector 4 A4/A5/ARC600 (XY Burst) - default level 1- default priority Low26 - ARC700 (Timer1) Low02
#define IV_IRQ4_ADDRESS (4*8)  //  Interrupt vector 4 A4/A5/ARC600 (XY Burst) - default level 1- default priority Low26 - ARC700 (Timer1) Low02
#define IV_IRQ4_NUM (4)  //  Interrupt vector 4 A4/A5/ARC600 (XY Burst) - default level 1- default priority Low26 - ARC700 (Timer1) Low02

// NOTE: ARC700 (Core version 0x31 and higher) swapped the XY burst and timer1 locations
#if (_ARCVER >= 0x31)
#define IV_TIMER1_NUM (4)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_TIMER1_ADDRESS (IV_TIMER1_NUM*8)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_TIMER1 (IV_TIMER1_ADDRESS)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_XY_BURST_COMPLETE_NUM (7)  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_XY_BURST_COMPLETE_ADDRESS (IV_XY_BURST_COMPLETE_NUM*8 )  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_XY_BURST_COMPLETE (IV_XY_BURST_COMPLETE_ADDRESS)  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#else // Core versions prior to ARC700 - A4/A5/ARC600 and earlier
#define IV_TIMER1_NUM (7)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_TIMER1_ADDRESS (IV_TIMER1_NUM*8)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_TIMER1 (IV_TIMER1_ADDRESS)  //  Interrupt vector 4 - ARC700 (Timer1) Low02
#define IV_XY_BURST_COMPLETE_NUM (4)  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_XY_BURST_COMPLETE_ADDRESS (IV_XY_BURST_COMPLETE_NUM*8 )  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_XY_BURST_COMPLETE (IV_XY_BURST_COMPLETE_ADDRESS)  //  Interrupt vector 7 - (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#endif


#define IV_IRQ5 (5*8)  //  Interrupt vector 5 (UART) - default level 1- default priority Low25 - ARC700 (UART) Low03
#define IV_IRQ5_ADDRESS (5*8)  //  Interrupt vector 5 (UART) - default level 1- default priority Low25 - ARC700 (UART) Low03
#define IV_IRQ5_NUM (5)  //  Interrupt vector 5 (UART) - default level 1- default priority Low25 - ARC700 (UART) Low03

#define IV_UART_NUM (5)
#define IV_UART_ADDRESS (IV_IRQ5*8)
#define IV_UART IV_UART_ADDRESS

#define IV_IRQ6 (6*8)  //  Interrupt vector 6 (EMAC) - default level 2 (ARC700 level 1) - default priority Med2 - ARC700 Low04
#define IV_IRQ6_ADDRESS (6*8)  //  Interrupt vector 6 (EMAC) - default level 2 (ARC700 level 1) - default priority Med2 - ARC700 Low04
#define IV_IRQ6_NUM (6)  //  Interrupt vector 6 (EMAC) - default level 2 (ARC700 level 1) - default priority Med2 - ARC700 Low04



#if (_ARCVER < 0x21)
#define IV_EMAC_NUM (6) // NOTE: Later builds from ARChitect are using IV_IRQ16
#define IV_EMAC_ADDRESS (IV_EMAC_NUM *8) // NOTE: Later builds from ARChitect are using IV_IRQ16
#define IV_EMAC (IV_EMAC_ADDRESS) // NOTE: Later builds from ARChitect are using IV_IRQ16
#endif

#define IV_IRQ7 (7*8)  //  Interrupt vector 7 A4/A5/ARC600(Timer 1) - default level 2 (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_IRQ7_ADDRESS (7*8)  //  Interrupt vector 7 A4/A5/ARC600(Timer 1) - default level 2 (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05
#define IV_IRQ7 (7*8)  //  Interrupt vector 7 A4/A5/ARC600(Timer 1) - default level 2 (ARC700 level 1) - default priority Med1 - ARC700 (XY Burst) Low05

#define IV_IRQ8 (8*8)  //  Interrupt vector 8 - default level 1- default priority Low24 - ARC700 Low04

#define IV_IRQ9 (9*8)  //  Interrupt vector 9 - default level 1- default priority Low23 - ARC700 Low05

#define IV_IRQ10 (10*8)  //  Interrupt vector 10 - default level 1- default priority Low22 - ARC700 Low06

#define IV_IRQ11 (11*8)  //  Interrupt vector 11 - default level 1- default priority Low21 - ARC700 Low07

#define IV_IRQ12 (12*8)  //  Interrupt vector 12 - default level 1- default priority Low20 - ARC700 Low08

#define IV_IRQ13 (13*8)  //  Interrupt vector 13 - default level 1- default priority Low19 - ARC700 Low09

#define IV_IRQ14 (14*8)  //  Interrupt vector 14 - default level 1- default priority Low18 - ARC700 Low10

#define IV_IRQ15 (15*8)  //  Interrupt vector 15 - default level 1- default priority Low17 - ARC700 Low11

#define IV_IRQ16 (16*8)  //  Interrupt vector 16 - default level 1- default priority Low16 - ARC700 Low12


#if (_ARCVER >= 0x21)
#define IV_EMAC_NUM (16) // NOTE: Later builds from ARChitect are using IV_IRQ16
#define IV_EMAC_ADDRESS (IV_EMAC_NUM *8) // NOTE: Later builds from ARChitect are using IV_IRQ16
#define IV_EMAC (IV_EMAC_ADDRESS) // NOTE: Later builds from ARChitect are using IV_IRQ16
#endif

#define IV_IRQ17 (17*8)  //  Interrupt vector 17 - default level 1- default priority Low15 - ARC700 Low13

#define IV_IRQ18 (18*8)  //  Interrupt vector 18 - default level 1- default priority Low14 - ARC700 Low14

#define IV_IRQ19 (19*8)  //  Interrupt vector 19 - default level 1- default priority Low13 - ARC700 Low15

#define IV_IRQ20 (20*8)  //  Interrupt vector 20 - default level 1- default priority Low12 - ARC700 Low16

#define IV_IRQ21 (21*8)  //  Interrupt vector 21 - default level 1- default priority Low11 - ARC700 Low17

#define IV_IRQ22 (22*8)  //  Interrupt vector 22 - default level 1- default priority Low10 - ARC700 Low18

#define IV_IRQ23 (23*8)  //  Interrupt vector 23 - default level 1- default priority Low09 - ARC700 Low19

#define IV_IRQ24 (24*8)  //  Interrupt vector 24 - default level 1- default priority Low08 - ARC700 Low20

#define IV_IRQ25 (25*8)  //  Interrupt vector 25 - default level 1- default priority Low07 - ARC700 Low21

#define IV_IRQ26 (26*8)  //  Interrupt vector 26 - default level 1- default priority Low06 - ARC700 Low22

#define IV_IRQ27 (27*8)  //  Interrupt vector 27 - default level 1- default priority Low05 - ARC700 Low23

#define IV_IRQ28 (28*8)  //  Interrupt vector 28 - default level 1- default priority Low04 - ARC700 Low24

#define IV_IRQ29 (29*8)  //  Interrupt vector 29 - default level 1- default priority Low03 - ARC700 Low25

#define IV_IRQ30 (30*8)  //  Interrupt vector 30 - default level 1- default priority Low02 - ARC700 Low26

#define IV_IRQ31 (31*8)  //  Interrupt vector 31 - default level 1- default priority Low01 - ARC700 Low27





// ARC700 Exception vectors, codes and exception cause registers.

//  Machine Check - Address 0x100 = vector 32
#define EV_MACH_CHECK_NUM (32) // Exception Vector Machine Check
#define EV_MACH_CHECK_ADDRESS (EV_MACH_CHECK_NUM *8) // Exception Vector Machine Check
#define EV_MACH_CHECK (EV_MACH_CHECK_ADDRESS) // Exception Vector Machine Check

#define  EV_MACH_CHK_IFETCH_DBL_FAULT_CODE (1) // Exception Machine Check - Instruction bFetch Double Fault Code
#define  EV_MACH_CHK_IFETCH_DBL_FAULT_ECR (0x200000 ) // Exception Machine Check - Instruction Fetch Double Fault Exception Cause Register
#define  EV_MACH_CHK_TLB_OVERLAP_CODE (1) // Exception Machine Check - TLB Overlap Entries Code
#define  EV_MACH_CHK_TLB_OVERLAP_ECR (0x200100 ) // Exception Machine Check - TLB Overlap Entries Exception Cause Register
#define  EV_MACH_CHK_TLB_FATAL_CODE (2) // Exception Machine Check - TLB Fatal Code
#define  EV_MACH_CHK_TLB_FATAL_ECR (0x200200 ) // Exception Machine Check - TLB Fatal Exception Cause Register
#define  EV_MACH_CHK_CACHE_FATAL_CODE (3) // Exception Machine Check - Cache Fatal Code
#define  EV_MACH_CHK_CACHE_FATAL_ECR (0x200300 ) // Exception Machine Check - Cache Fatal Exception Cause Register
#define  EV_MACH_CHK_KERNEL_MEM_ERR_CODE (4) // Exception Machine Check - Kernel Memory Error Code
#define  EV_MACH_CHK_KERNEL_MEM_ERR_ECR (0x200400 ) // Exception Machine Check - Kernel Memory Error Exception Cause Register
#define  EV_MACH_CHK_DCACHE_FLUSH_ERR_CODE (5) // Exception Machine Check - Data Cache Flush Memory Error Code
#define  EV_MACH_CHK_DCACHE_FLUSH_ERR_ECR (0x200500 ) // Exception Machine Check - Data Cache Flush Memory Error - Exception Cause Register

//  Instruction Fetch TLB Miss - Address 0x108 = vector 33

#define  EV_IFETCH_TLB_MISS_NUM (33 ) // Exception Instruction Fetch TLB Miss
#define  EV_IFETCH_TLB_MISS_ADDRESS (EV_IFETCH_TLB_MISS_NUM *8) // Exception Instruction Fetch TLB Miss
#define  EV_IFETCH_TLB_MISS (EV_IFETCH_TLB_MISS_ADDRESS) // Exception Instruction Fetch TLB Miss

#define  EV_IFETCH_TLB_MISS_CODE (0) // Exception Instruction Fetch TLB Miss Code
#define  EV_IFETCH_TLB_MISS_ECR (0x210000) // Exception Instruction Fetch TLB Miss - Exception Cause Register


//  Data TLB Miss - Address 0x110 = vector 34

#define  EV_DATA_TLB_MISS_NUM (34) // Exception Data TLB Miss
#define  EV_DATA_TLB_MISS_ADDRESS (EV_DATA_TLB_MISS_NUM*8) // Exception Data TLB Miss
#define  EV_DATA_TLB_MISS (EV_DATA_TLB_MISS_ADDRESS) // Exception Data TLB Miss

#define  EV_DATA_TLB_MISS_LD_CODE (1) // Exception Data TLB Miss - Load - Code
#define  EV_DATA_TLB_MISS_LD_ECR (0x220100) // Exception Data TLB Miss - Load - Exception Cause Register

#define  EV_DATA_TLB_MISS_ST_CODE (2) // Exception Data TLB Miss - Store - Code
#define  EV_DATA_TLB_MISS_ST_ECR (0x220200) // Exception Data TLB Miss - Store - Exception Cause Register

#define  EV_DATA_TLB_MISS_EX_CODE (3) // Exception Data TLB Miss - Extend - Code
#define  EV_DATA_TLB_MISS_EX_ECR (0x220300) // Exception Data TLB Miss - Extend - Exception Cause Register


//  MMU TLB Protection Violation Exception - Address 0x118 = vector 35

#define EV_TLB_PROT_VIOLATION_NUM (35) // Exception MMU TLB Protection Violation
#define EV_TLB_PROT_VIOLATION_ADDRESS (EV_TLB_PROT_VIOLATION_NUM*8 ) // Exception MMU TLB Protection Violation
#define EV_TLB_PROT_VIOLATION (EV_TLB_PROT_VIOLATION_ADDRESS ) // Exception MMU TLB Protection Violation

#define EV_TLB_PROT_VIOLATION_IFETCH_CODE (0) // Exception MMU TLB Protection Violation - Instruction Fetch - Code
#define EV_TLB_PROT_VIOLATION_IFETCH_ECR (0x230000) // Exception MMU TLB Protection Violation - Instruction Fetch - Exception Cause Register
#define EV_TLB_PROT_VIOLATION_DATA_LD_CODE (1) // Exception MMU TLB Protection Violation - Data Load - Code
#define EV_TLB_PROT_VIOLATION_DATA_LD_ECR (0x230100) // Exception MMU TLB Protection Violation - Data Load - Exception Cause Register
#define EV_TLB_PROT_VIOLATION_DATA_ST_CODE (2) // Exception MMU TLB Protection Violation - Data Store - Code
#define EV_TLB_PROT_VIOLATION_DATA_ST_ECR (0x230200) // Exception MMU TLB Protection Violation - Data Store - Exception Cause Register
#define EV_TLB_PROT_VIOLATION_DATA_EX_CODE (3) // Exception MMU TLB Protection Violation - Data Extend - Code
#define EV_TLB_PROT_VIOLATION_DATA_EX_ECR (0x230300) // Exception MMU TLB Protection Violation - Data Extend - Exception Cause Register

//  Privilege Violation Exception - Address 0x120 = vector 36

#define EV_PRIV_VIOLATION_NUM (36) // Exception Vector Privilege Violation
#define EV_PRIV_VIOLATION_ADDRESS (EV_PRIV_VIOLATION_NUM*8 ) // Exception Vector Privilege Violation
#define EV_PRIV_VIOLATION (EV_PRIV_VIOLATION_ADDRESS ) // Exception Vector Privilege Violation


#define EV_PRIV_VIOLATION_CODE (0) // Exception Vector Privilege Violation - Privilege Violation Code

#define EV_PRIV_VIOLATION_ECR (0x240000) // Exception Vector Privilege Violation - Privilege Violation - Exception Cause Register

#define EV_PRIV_VIOLATION_DISABLED_EXT_CODE (1) // Exception Vector Privilege Violation - Disabled Extension Code

#define EV_PRIV_VIOLATION_DISABLED_EXT_ECR_BASE (0x240100) // Exception Vector Privilege Violation - Disabled Extension - Exception Cause Register Base

#define EV_PRIV_VIOLATION_AP_HIT_MEM_REG_CODE (2) // Exception Vector Privilege Violation - ActionPoint Hit (Memory or Register) Code

#define EV_PRIV_VIOLATION_AP_HIT_MEM_REG_ECR_BASE (0x240200) // Exception Vector Privilege Violation - ActionPoint Hit (Memory or Register) - Exception Cause Register Base

#define EV_PRIV_VIOLATION_AP_HIT_CODE (6) // Exception Vector Privilege Violation - ActionPoint Hit Code

#define EV_PRIV_VIOLATION_AP_HIT_ECR_BASE (0x200600 ) // Exception Vector Privilege Violation - ActionPoint Hit Exception Cause Register Base


//  Trap Exception  - Address 0x128 = vector 37

#define EV_TRAP_NUM  (37) // Exception Vector Trap Instruction
#define EV_TRAP_ADDRESS  (EV_TRAP_NUM  *8) // Exception Vector Trap Instruction
#define EV_TRAP  (EV_TRAP_ADDRESS  ) // Exception Vector Trap Instruction

#define EV_TRAP_CODE (0) // Exception Vector Privilege Violation - Trap Code

#define EV_TRAP_ECR_BASE (0x240000) // Exception Vector Trap  - Exception Cause Register Base

//  Extension Instruction - Address 0x130 = vector 38

#define EV_EXT_INSTRUCTION_NUM  (38) // Exception Vector Extension Instruction
#define EV_EXT_INSTRUCTION_ADDRESS  (EV_EXT_INSTRUCTION_NUM*8) // Exception Vector Extension Instruction
#define EV_EXT_INSTRUCTION  (EV_EXT_INSTRUCTION_ADDRESS) // Exception Vector Extension Instruction

#define EV_EXT_INSTRUCTION_ECR_BASE (0x260000) // Exception Vector Extension Instruction - Exception Cause Regisister base

// EM XY registers
#if defined __Xxy && defined __Xagu && _ARCVER >= 0x40

#define AGU_AUX_MOD0 __AGU_AUX_MOD0
#define AGU_AUX_MOD1 __AGU_AUX_MOD1
#define AGU_AUX_MOD2 __AGU_AUX_MOD2
#define AGU_AUX_MOD3 __AGU_AUX_MOD3

#define AGU_AUX_AP0 __AGU_AUX_AP0
#define AGU_AUX_AP1 __AGU_AUX_AP1
#define AGU_AUX_AP2 __AGU_AUX_AP2
#define AGU_AUX_AP3 __AGU_AUX_AP3

#define AGU_AUX_OS0 __AGU_AUX_OS0
#define AGU_AUX_OS1 __AGU_AUX_OS1

#if defined(__Xagu_medium) || defined(__Xagu_large)
// AGU medium: 12 MOD, 8 AP, 4 OS

#define AGU_AUX_MOD4 __AGU_AUX_MOD4
#define AGU_AUX_MOD5 __AGU_AUX_MOD5
#define AGU_AUX_MOD6 __AGU_AUX_MOD6
#define AGU_AUX_MOD7 __AGU_AUX_MOD7
#define AGU_AUX_MOD8 __AGU_AUX_MOD8
#define AGU_AUX_MOD9 __AGU_AUX_MOD9
#define AGU_AUX_MOD10 __AGU_AUX_MOD10
#define AGU_AUX_MOD11 __AGU_AUX_MOD11

#define AGU_AUX_AP4 __AGU_AUX_AP4
#define AGU_AUX_AP5 __AGU_AUX_AP5
#define AGU_AUX_AP6 __AGU_AUX_AP6
#define AGU_AUX_AP7 __AGU_AUX_AP7

#define AGU_AUX_OS2 __AGU_AUX_OS2
#define AGU_AUX_OS3 __AGU_AUX_OS3

#if defined(__Xagu_large)
// AGU large:  24 MOD, 12 AP, 8 OS

#define AGU_AUX_MOD12 __AGU_AUX_MOD12
#define AGU_AUX_MOD13 __AGU_AUX_MOD13
#define AGU_AUX_MOD14 __AGU_AUX_MOD14
#define AGU_AUX_MOD15 __AGU_AUX_MOD15
#define AGU_AUX_MOD16 __AGU_AUX_MOD16
#define AGU_AUX_MOD17 __AGU_AUX_MOD17
#define AGU_AUX_MOD18 __AGU_AUX_MOD18
#define AGU_AUX_MOD19 __AGU_AUX_MOD19
#define AGU_AUX_MOD20 __AGU_AUX_MOD20
#define AGU_AUX_MOD21 __AGU_AUX_MOD21
#define AGU_AUX_MOD22 __AGU_AUX_MOD22
#define AGU_AUX_MOD23 __AGU_AUX_MOD23

#define AGU_AUX_AP8 __AGU_AUX_AP8
#define AGU_AUX_AP9 __AGU_AUX_AP9
#define AGU_AUX_AP10 __AGU_AUX_AP10
#define AGU_AUX_AP11 __AGU_AUX_AP11
#define AGU_AUX_AP12 __AGU_AUX_AP12
#define AGU_AUX_AP13 __AGU_AUX_AP13
#define AGU_AUX_AP14 __AGU_AUX_AP14
#define AGU_AUX_AP15 __AGU_AUX_AP15

#define AGU_AUX_OS4 __AGU_AUX_OS4
#define AGU_AUX_OS5 __AGU_AUX_OS5
#define AGU_AUX_OS6 __AGU_AUX_OS6
#define AGU_AUX_OS7 __AGU_AUX_OS7

#endif // defined(__Xagu_large)
#endif // defined(__Xagu_medium) || defined(__Xagu_large)
#endif // defined __Xxy && defined __Xagu && _ARCVER >= 0x40

/*****************************************************************

Revsion history

03 AUG 2006 Corrected auxilliary short range discription to show
            ARCtangent-A5 more limited ranges.  ARC700 Exception
            vectors were lacking hex format seen as incorrect decimal.
    Converted to correct decimal

28 JUL 2006 Added MACROS START_TIMER(),START_TIMER0(),START_TIMER1(),
            GET_TIMER1()

10 JUL 2006 Corrections to ARC700 Privilage Violation and Trap exception
            defines.

04 JUN 2006 Add _ADDRESS and _NUM forms for Interrupt and exception
            locations to make it more clear.  Left the older non-suffix
            forms which were addresses.

03 JUN 2006 Make it clear that from that ARC700 swapped the Timer1 and
            XY memory interrupts from 04 for XY and 07 for timer1 to the
            opposite from prevoius cores.  Added descriptive names for
            interrupts that are normally designated for standard hardware.

17 MAR 2006 Duplicate interrupt vector address listing removed.

04 MAR 2006 Corrected burst fill macro with non existant define.
            Added better burst macro names
        Added bit setting defines for XY memory burst region (0=X)
            and bank (0=bank1 etc). Added conditional include

22 FEB 2006 7.4.3 compilers with patch #1 or later tools allow use
            of -Hasmcpp and $reg(N) to convert a defined number to
            a register name

07 JAN 2006 EV_TRAP and last line comment errors removed

01 JAN 2006 Added interrupt and exception vectors (including ARC700)

11 NOV 2009 Comment reformatting...

30 JAN 2015 EM XY registers added

******************************************************************/

//lint -restore
#endif // #ifndef _ARC_REG_H_
