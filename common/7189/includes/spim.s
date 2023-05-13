;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; @file       common/7189/includes/spim.s
;;;
;;; @project    EM7189
;;;
;;; @brief      <DESCRIPTION>
;;;
;;; @classification  Confidential
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; @copyright Copyright (C) 2015-2018 EM Microelectronic
;;; @cond
;;;
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 1. Redistributions of source code must retain the above copyright notice,
;;; this list of conditions and the following disclaimer.
;;; 2. Redistributions in binary form must reproduce the above copyright notice,
;;; this list of conditions and the following disclaimer in the documentation
;;; and/or other materials provided with the distribution.
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;; @endcond
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.ifndef SPIM_S
.define SPIM_S, 1


.define REG_SPIM0_CONFIG, 0xf00300
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SHIFT, 0
.define     SPIM0_CONFIG_SPIM_CLK_SEL_MASK,  0x7
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC, 0x0
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_2, 0x1
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_3, 0x2
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_4, 0x3
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_6, 0x4
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_8, 0x5
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_16, 0x6
.define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_32, 0x7

.define     SPIM0_CONFIG_SPIM_CPHA_SHIFT, 3
.define     SPIM0_CONFIG_SPIM_CPHA_MASK,  0x8
.define     SPIM0_CONFIG_SPIM_CPOL_SHIFT, 4
.define     SPIM0_CONFIG_SPIM_CPOL_MASK,  0x10
.define     SPIM0_CONFIG_SPIM_MSB_SHIFT, 5
.define     SPIM0_CONFIG_SPIM_MSB_MASK,  0x20
.define     SPIM0_CONFIG_SPIM_ICFG_SHIFT, 6
.define     SPIM0_CONFIG_SPIM_ICFG_MASK,  0x40
.define     SPIM0_CONFIG_SPIM_ICFG_4_WIRE, 0x0
.define     SPIM0_CONFIG_SPIM_ICFG_3_WIRE, 0x1

.define     SPIM0_CONFIG_RESERVED_7_7_SHIFT, 7
.define     SPIM0_CONFIG_RESERVED_7_7_MASK,  0x80
.define     SPIM0_CONFIG_SPIM_EN_SHIFT, 8
.define     SPIM0_CONFIG_SPIM_EN_MASK,  0x100
.define     SPIM0_CONFIG_RESERVED_15_9_SHIFT, 9
.define     SPIM0_CONFIG_RESERVED_15_9_MASK,  0xfe00
.define     SPIM0_CONFIG_SPIM_RW_SEL_SHIFT, 16
.define     SPIM0_CONFIG_SPIM_RW_SEL_MASK,  0x10000
.define     SPIM0_CONFIG_SPIM_RW_SEL_WRITE, 0x0
.define     SPIM0_CONFIG_SPIM_RW_SEL_READ, 0x1

.define     SPIM0_CONFIG_RESERVED_31_17_SHIFT, 17
.define     SPIM0_CONFIG_RESERVED_31_17_MASK,  0xfffe0000

.define REG_SPIM0_CONTROL, 0xf00304
.define     SPIM0_CONTROL_SPIM_DAT_LNGTH_SHIFT, 0
.define     SPIM0_CONTROL_SPIM_DAT_LNGTH_MASK,  0x1f
.define     SPIM0_CONTROL_RESERVED_7_5_SHIFT, 5
.define     SPIM0_CONTROL_RESERVED_7_5_MASK,  0xe0
.define     SPIM0_CONTROL_SPIM_CMD_LNGTH_SHIFT, 8
.define     SPIM0_CONTROL_SPIM_CMD_LNGTH_MASK,  0x700
.define     SPIM0_CONTROL_RESERVED_15_11_SHIFT, 11
.define     SPIM0_CONTROL_RESERVED_15_11_MASK,  0xf800
.define     SPIM0_CONTROL_SPIM_START_SHIFT, 16
.define     SPIM0_CONTROL_SPIM_START_MASK,  0x10000
.define     SPIM0_CONTROL_SPIM_STOP_SHIFT, 17
.define     SPIM0_CONTROL_SPIM_STOP_MASK,  0x20000
.define     SPIM0_CONTROL_RESERVED_30_18_SHIFT, 18
.define     SPIM0_CONTROL_RESERVED_30_18_MASK,  0x7ffc0000
.define     SPIM0_CONTROL_SPIM_RESET_SHIFT, 31
.define     SPIM0_CONTROL_SPIM_RESET_MASK,  0x80000000

.define REG_SPIM0_STATUS, 0xf00308
.define     SPIM0_STATUS_SPIM_BUSY_SHIFT, 0
.define     SPIM0_STATUS_SPIM_BUSY_MASK,  0x1
.define     SPIM0_STATUS_SPIM_CMD_SENT_SHIFT, 1
.define     SPIM0_STATUS_SPIM_CMD_SENT_MASK,  0x2
.define     SPIM0_STATUS_SPIM_DAT_SENT_SHIFT, 2
.define     SPIM0_STATUS_SPIM_DAT_SENT_MASK,  0x4
.define     SPIM0_STATUS_RESERVED_3_3_SHIFT, 3
.define     SPIM0_STATUS_RESERVED_3_3_MASK,  0x8
.define     SPIM0_STATUS_SPIM_STOP_STAT_SHIFT, 4
.define     SPIM0_STATUS_SPIM_STOP_STAT_MASK,  0x10
.define     SPIM0_STATUS_RESERVED_7_5_SHIFT, 5
.define     SPIM0_STATUS_RESERVED_7_5_MASK,  0xe0
.define     SPIM0_STATUS_SPIM_DATA_NUM_SHIFT, 8
.define     SPIM0_STATUS_SPIM_DATA_NUM_MASK,  0x1f00
.define     SPIM0_STATUS_RESERVED_15_13_SHIFT, 13
.define     SPIM0_STATUS_RESERVED_15_13_MASK,  0xe000
.define     SPIM0_STATUS_SPIM_CMD_NUM_SHIFT, 16
.define     SPIM0_STATUS_SPIM_CMD_NUM_MASK,  0x70000
.define     SPIM0_STATUS_RESERVED_31_19_SHIFT, 19
.define     SPIM0_STATUS_RESERVED_31_19_MASK,  0xfff80000

.define REG_SPIM0_TX_CMD, 0xf0030c ; This register holds the command to be transmitted and, optionally, a target address.

.define REG_SPIM0_TX_BUFFER0, 0xf00310 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM0_TX_BUFFER1, 0xf00314 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM0_TX_BUFFER2, 0xf00318 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM0_TX_BUFFER3, 0xf0031c ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM0_RX_CMD, 0xf00320 ; This register holds the data received during command transmission.

.define REG_SPIM0_RX_BUFFER0, 0xf00324 ; This register holds the data received during read transactions.

.define REG_SPIM0_RX_BUFFER1, 0xf00328 ; This register holds the data received during read transactions.

.define REG_SPIM0_RX_BUFFER2, 0xf0032c ; This register holds the data received during read transactions.

.define REG_SPIM0_RX_BUFFER3, 0xf00330 ; This register holds the data received during read transactions.


.define REG_SPIM1_CONFIG, 0xf00400
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SHIFT, 0
.define     SPIM1_CONFIG_SPIM_CLK_SEL_MASK,  0x7
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC, 0x0
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_2, 0x1
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_3, 0x2
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_4, 0x3
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_6, 0x4
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_8, 0x5
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_16, 0x6
.define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_32, 0x7

.define     SPIM1_CONFIG_SPIM_CPHA_SHIFT, 3
.define     SPIM1_CONFIG_SPIM_CPHA_MASK,  0x8
.define     SPIM1_CONFIG_SPIM_CPOL_SHIFT, 4
.define     SPIM1_CONFIG_SPIM_CPOL_MASK,  0x10
.define     SPIM1_CONFIG_SPIM_MSB_SHIFT, 5
.define     SPIM1_CONFIG_SPIM_MSB_MASK,  0x20
.define     SPIM1_CONFIG_SPIM_ICFG_SHIFT, 6
.define     SPIM1_CONFIG_SPIM_ICFG_MASK,  0x40
.define     SPIM1_CONFIG_SPIM_ICFG_4_WIRE, 0x0
.define     SPIM1_CONFIG_SPIM_ICFG_3_WIRE, 0x1

.define     SPIM1_CONFIG_RESERVED_7_7_SHIFT, 7
.define     SPIM1_CONFIG_RESERVED_7_7_MASK,  0x80
.define     SPIM1_CONFIG_SPIM_EN_SHIFT, 8
.define     SPIM1_CONFIG_SPIM_EN_MASK,  0x100
.define     SPIM1_CONFIG_RESERVED_15_9_SHIFT, 9
.define     SPIM1_CONFIG_RESERVED_15_9_MASK,  0xfe00
.define     SPIM1_CONFIG_SPIM_RW_SEL_SHIFT, 16
.define     SPIM1_CONFIG_SPIM_RW_SEL_MASK,  0x10000
.define     SPIM1_CONFIG_SPIM_RW_SEL_WRITE, 0x0
.define     SPIM1_CONFIG_SPIM_RW_SEL_READ, 0x1

.define     SPIM1_CONFIG_RESERVED_31_17_SHIFT, 17
.define     SPIM1_CONFIG_RESERVED_31_17_MASK,  0xfffe0000

.define REG_SPIM1_CONTROL, 0xf00404
.define     SPIM1_CONTROL_SPIM_DAT_LNGTH_SHIFT, 0
.define     SPIM1_CONTROL_SPIM_DAT_LNGTH_MASK,  0x1f
.define     SPIM1_CONTROL_RESERVED_7_5_SHIFT, 5
.define     SPIM1_CONTROL_RESERVED_7_5_MASK,  0xe0
.define     SPIM1_CONTROL_SPIM_CMD_LNGTH_SHIFT, 8
.define     SPIM1_CONTROL_SPIM_CMD_LNGTH_MASK,  0x700
.define     SPIM1_CONTROL_RESERVED_15_11_SHIFT, 11
.define     SPIM1_CONTROL_RESERVED_15_11_MASK,  0xf800
.define     SPIM1_CONTROL_SPIM_START_SHIFT, 16
.define     SPIM1_CONTROL_SPIM_START_MASK,  0x10000
.define     SPIM1_CONTROL_SPIM_STOP_SHIFT, 17
.define     SPIM1_CONTROL_SPIM_STOP_MASK,  0x20000
.define     SPIM1_CONTROL_RESERVED_30_18_SHIFT, 18
.define     SPIM1_CONTROL_RESERVED_30_18_MASK,  0x7ffc0000
.define     SPIM1_CONTROL_SPIM_RESET_SHIFT, 31
.define     SPIM1_CONTROL_SPIM_RESET_MASK,  0x80000000

.define REG_SPIM1_STATUS, 0xf00408
.define     SPIM1_STATUS_SPIM_BUSY_SHIFT, 0
.define     SPIM1_STATUS_SPIM_BUSY_MASK,  0x1
.define     SPIM1_STATUS_SPIM_CMD_SENT_SHIFT, 1
.define     SPIM1_STATUS_SPIM_CMD_SENT_MASK,  0x2
.define     SPIM1_STATUS_SPIM_DAT_SENT_SHIFT, 2
.define     SPIM1_STATUS_SPIM_DAT_SENT_MASK,  0x4
.define     SPIM1_STATUS_RESERVED_3_3_SHIFT, 3
.define     SPIM1_STATUS_RESERVED_3_3_MASK,  0x8
.define     SPIM1_STATUS_SPIM_STOP_STAT_SHIFT, 4
.define     SPIM1_STATUS_SPIM_STOP_STAT_MASK,  0x10
.define     SPIM1_STATUS_RESERVED_7_5_SHIFT, 5
.define     SPIM1_STATUS_RESERVED_7_5_MASK,  0xe0
.define     SPIM1_STATUS_SPIM_DATA_NUM_SHIFT, 8
.define     SPIM1_STATUS_SPIM_DATA_NUM_MASK,  0x1f00
.define     SPIM1_STATUS_RESERVED_15_13_SHIFT, 13
.define     SPIM1_STATUS_RESERVED_15_13_MASK,  0xe000
.define     SPIM1_STATUS_SPIM_CMD_NUM_SHIFT, 16
.define     SPIM1_STATUS_SPIM_CMD_NUM_MASK,  0x70000
.define     SPIM1_STATUS_RESERVED_31_19_SHIFT, 19
.define     SPIM1_STATUS_RESERVED_31_19_MASK,  0xfff80000

.define REG_SPIM1_TX_CMD, 0xf0040c ; This register holds the command to be transmitted and, optionally, a target address.

.define REG_SPIM1_TX_BUFFER0, 0xf00410 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM1_TX_BUFFER1, 0xf00414 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM1_TX_BUFFER2, 0xf00418 ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM1_TX_BUFFER3, 0xf0041c ; This register holds the data to be transmitted during write transactions.

.define REG_SPIM1_RX_CMD, 0xf00420 ; This register holds the data received during command transmission.

.define REG_SPIM1_RX_BUFFER0, 0xf00424 ; This register holds the data received during read transactions.

.define REG_SPIM1_RX_BUFFER1, 0xf00428 ; This register holds the data received during read transactions.

.define REG_SPIM1_RX_BUFFER2, 0xf0042c ; This register holds the data received during read transactions.

.define REG_SPIM1_RX_BUFFER3, 0xf00430 ; This register holds the data received during read transactions.


.endif /* !SPIM_S */
