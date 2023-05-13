; CONFIDENTIAL AND PROPRIETARY INFORMATION
; Copyright 2002-2005 ARC International (Unpublished)
; All Rights Reserved.
;
; This document, material and/or software contains confidential
; and proprietary information of ARC International and is
; protected by copyright, trade secret and other state, federal,
; and international laws, and may be embodied in patents issued
; or pending.  Its receipt or possession does not convey any
; rights to use, reproduce, disclose its contents, or to
; manufacture, or sell anything it may describe.  Reverse
; engineering is prohibited, and reproduction, disclosure or use
; without specific written authorization of ARC International is
; strictly forbidden.  ARC and the ARC logotype are trademarks of
; ARC International.
;
; ARC Product:  ARCv2EM v1.1.14
; File version:  IP Library version 1.1.14, file revision
; ARC Chip ID:  9999
;
; Description:
;

; Assembler directives for eia extensions in this design
.set apex_extensioninstructions_present,1
.extInstruction crc_32, 7, 6, SUFFIX_COND, SYNTAX_3OP
.set apex_com_arc_hardware_floating_point_unit_fpu_present,1
.extAuxRegister fpu_build,0xc8,r
.extAuxRegister fpu_ctrl,0x300,r|w
.extAuxRegister fpu_status,0x301,r|w
.extInstruction fsmul,6,0,SUFFIX_COND,SYNTAX_3OP
.extInstruction fsadd,6,1,SUFFIX_COND,SYNTAX_3OP
.extInstruction fssub,6,2,SUFFIX_COND,SYNTAX_3OP
.extInstruction fcvt32,6,8,SUFFIX_COND,SYNTAX_3OP
.extInstruction fsdiv,6,7,SUFFIX_COND,SYNTAX_3OP
.extInstruction fscmp,6,3,SUFFIX_COND|SUFFIX_FLAG,SYNTAX_3OP
.extInstruction fscmpf,6,4,SUFFIX_COND|SUFFIX_FLAG,SYNTAX_3OP
.extInstruction fssqrt,6,0,FLAGS_NONE,SYNTAX_2OP
