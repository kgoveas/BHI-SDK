;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; @file       common/7189/includes/i2cm.s
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

.ifndef I2CM_S
.define I2CM_S, 1


.define REG_I2CM0_CONTROL, 0xf00500
.define     I2CM0_CONTROL_REG_ADDR_SHIFT, 0
.define     I2CM0_CONTROL_REG_ADDR_MASK,  0xff
.define     I2CM0_CONTROL_SLV_ADDR_SHIFT, 8
.define     I2CM0_CONTROL_SLV_ADDR_MASK,  0x7f00
.define     I2CM0_CONTROL_RW_SEL_SHIFT, 15
.define     I2CM0_CONTROL_RW_SEL_MASK,  0x8000
.define     I2CM0_CONTROL_TRANS_LENGTH_SHIFT, 16
.define     I2CM0_CONTROL_TRANS_LENGTH_MASK,  0x1f0000
.define     I2CM0_CONTROL_NO_STOP_SHIFT, 21
.define     I2CM0_CONTROL_NO_STOP_MASK,  0x200000
.define     I2CM0_CONTROL_RESERVED_22_SHIFT, 22
.define     I2CM0_CONTROL_RESERVED_22_MASK,  0x400000
.define     I2CM0_CONTROL_REG_ADDR_EN_SHIFT, 23
.define     I2CM0_CONTROL_REG_ADDR_EN_MASK,  0x800000
.define     I2CM0_CONTROL_START_TRANS_SHIFT, 24
.define     I2CM0_CONTROL_START_TRANS_MASK,  0x1000000
.define     I2CM0_CONTROL_STOP_TRANS_SHIFT, 25
.define     I2CM0_CONTROL_STOP_TRANS_MASK,  0x2000000
.define     I2CM0_CONTROL_RESERVED_30_26_SHIFT, 26
.define     I2CM0_CONTROL_RESERVED_30_26_MASK,  0x7c000000
.define     I2CM0_CONTROL_I2CM_RESET_SHIFT, 31
.define     I2CM0_CONTROL_I2CM_RESET_MASK,  0x80000000

.define REG_I2CM0_TIMING_CONFIG0, 0xf00504
.define     I2CM0_TIMING_CONFIG0_SCL_LOW_PER_SHIFT, 0
.define     I2CM0_TIMING_CONFIG0_SCL_LOW_PER_MASK,  0xff
.define     I2CM0_TIMING_CONFIG0_SCL_HIGH_PER_SHIFT, 8
.define     I2CM0_TIMING_CONFIG0_SCL_HIGH_PER_MASK,  0xff00
.define     I2CM0_TIMING_CONFIG0_SCL_STOP_PER_SHIFT, 16
.define     I2CM0_TIMING_CONFIG0_SCL_STOP_PER_MASK,  0xff0000
.define     I2CM0_TIMING_CONFIG0_SCL_START_PER_SHIFT, 24
.define     I2CM0_TIMING_CONFIG0_SCL_START_PER_MASK,  0xff000000

.define REG_I2CM0_TIMING_CONFIG1, 0xf00508
.define     I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD_SHIFT, 0
.define     I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD_MASK,  0xff
.define     I2CM0_TIMING_CONFIG1_SDA_LOW_TIME_SHIFT, 8
.define     I2CM0_TIMING_CONFIG1_SDA_LOW_TIME_MASK,  0xff00
.define     I2CM0_TIMING_CONFIG1_SCL_RISE_TIME_SHIFT, 16
.define     I2CM0_TIMING_CONFIG1_SCL_RISE_TIME_MASK,  0xff0000
.define     I2CM0_TIMING_CONFIG1_RESERVED_31_24_SHIFT, 24
.define     I2CM0_TIMING_CONFIG1_RESERVED_31_24_MASK,  0xff000000

.define REG_I2CM0_CONFIG, 0xf0050c
.define     I2CM0_CONFIG_SDA_LOW_DET_DIS_SHIFT, 0
.define     I2CM0_CONFIG_SDA_LOW_DET_DIS_MASK,  0x1
.define     I2CM0_CONFIG_CK_STRETCH_EN_SHIFT, 1
.define     I2CM0_CONFIG_CK_STRETCH_EN_MASK,  0x2
.define     I2CM0_CONFIG_RESERVED_7_2_SHIFT, 2
.define     I2CM0_CONFIG_RESERVED_7_2_MASK,  0xfc
.define     I2CM0_CONFIG_I2CM_EN_SHIFT, 8
.define     I2CM0_CONFIG_I2CM_EN_MASK,  0x100
.define     I2CM0_CONFIG_RESERVED_31_9_SHIFT, 9
.define     I2CM0_CONFIG_RESERVED_31_9_MASK,  0xfffffe00

.define REG_I2CM0_STATUS, 0xf00510
.define     I2CM0_STATUS_BUSY_SHIFT, 0
.define     I2CM0_STATUS_BUSY_MASK,  0x1
.define     I2CM0_STATUS_CK_STRETCH_STAT_SHIFT, 1
.define     I2CM0_STATUS_CK_STRETCH_STAT_MASK,  0x2
.define     I2CM0_STATUS_NAK_SENT_SHIFT, 2
.define     I2CM0_STATUS_NAK_SENT_MASK,  0x4
.define     I2CM0_STATUS_DATA_TRANS_STAT_SHIFT, 3
.define     I2CM0_STATUS_DATA_TRANS_STAT_MASK,  0x8
.define     I2CM0_STATUS_STOP_STAT_SHIFT, 4
.define     I2CM0_STATUS_STOP_STAT_MASK,  0x10
.define     I2CM0_STATUS_NAK_RCVD_SHIFT, 5
.define     I2CM0_STATUS_NAK_RCVD_MASK,  0x20
.define     I2CM0_STATUS_REG_ADDR_SENT_SHIFT, 6
.define     I2CM0_STATUS_REG_ADDR_SENT_MASK,  0x40
.define     I2CM0_STATUS_RD_SEQ_ACT_SHIFT, 7
.define     I2CM0_STATUS_RD_SEQ_ACT_MASK,  0x80
.define     I2CM0_STATUS_WR_SEQ_ACT_SHIFT, 8
.define     I2CM0_STATUS_WR_SEQ_ACT_MASK,  0x100
.define     I2CM0_STATUS_START_SENT_SHIFT, 9
.define     I2CM0_STATUS_START_SENT_MASK,  0x200
.define     I2CM0_STATUS_SDA_LOW_STAT_SHIFT, 10
.define     I2CM0_STATUS_SDA_LOW_STAT_MASK,  0x400
.define     I2CM0_STATUS_START_TRANS_STAT_SHIFT, 11
.define     I2CM0_STATUS_START_TRANS_STAT_MASK,  0x800
.define     I2CM0_STATUS_STOP_TRANS_STAT_SHIFT, 12
.define     I2CM0_STATUS_STOP_TRANS_STAT_MASK,  0x1000
.define     I2CM0_STATUS_PAUSE_STAT_SHIFT, 13
.define     I2CM0_STATUS_PAUSE_STAT_MASK,  0x2000
.define     I2CM0_STATUS_RESERVED_15_14_SHIFT, 14
.define     I2CM0_STATUS_RESERVED_15_14_MASK,  0xc000
.define     I2CM0_STATUS_TRANS_LENGTH_SHIFT, 16
.define     I2CM0_STATUS_TRANS_LENGTH_MASK,  0x1f0000
.define     I2CM0_STATUS_RESERVED_31_21_SHIFT, 21
.define     I2CM0_STATUS_RESERVED_31_21_MASK,  0xffe00000

.define REG_I2CM0_TX_BUFFER0, 0xf00514 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM0_TX_BUFFER1, 0xf00518 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM0_TX_BUFFER2, 0xf0051c ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM0_TX_BUFFER3, 0xf00520 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM0_RX_BUFFER0, 0xf00524 ; This register holds the data received during read transactions.

.define REG_I2CM0_RX_BUFFER1, 0xf00528 ; This register holds the data received during read transactions.

.define REG_I2CM0_RX_BUFFER2, 0xf0052c ; This register holds the data received during read transactions.

.define REG_I2CM0_RX_BUFFER3, 0xf00530 ; This register holds the data received during read transactions.


.define REG_I2CM1_CONTROL, 0xf00600
.define     I2CM1_CONTROL_REG_ADDR_SHIFT, 0
.define     I2CM1_CONTROL_REG_ADDR_MASK,  0xff
.define     I2CM1_CONTROL_SLV_ADDR_SHIFT, 8
.define     I2CM1_CONTROL_SLV_ADDR_MASK,  0x7f00
.define     I2CM1_CONTROL_RW_SEL_SHIFT, 15
.define     I2CM1_CONTROL_RW_SEL_MASK,  0x8000
.define     I2CM1_CONTROL_TRANS_LENGTH_SHIFT, 16
.define     I2CM1_CONTROL_TRANS_LENGTH_MASK,  0x1f0000
.define     I2CM1_CONTROL_NO_STOP_SHIFT, 21
.define     I2CM1_CONTROL_NO_STOP_MASK,  0x200000
.define     I2CM1_CONTROL_RESERVED_22_SHIFT, 22
.define     I2CM1_CONTROL_RESERVED_22_MASK,  0x400000
.define     I2CM1_CONTROL_REG_ADDR_EN_SHIFT, 23
.define     I2CM1_CONTROL_REG_ADDR_EN_MASK,  0x800000
.define     I2CM1_CONTROL_START_TRANS_SHIFT, 24
.define     I2CM1_CONTROL_START_TRANS_MASK,  0x1000000
.define     I2CM1_CONTROL_STOP_TRANS_SHIFT, 25
.define     I2CM1_CONTROL_STOP_TRANS_MASK,  0x2000000
.define     I2CM1_CONTROL_RESERVED_30_26_SHIFT, 26
.define     I2CM1_CONTROL_RESERVED_30_26_MASK,  0x7c000000
.define     I2CM1_CONTROL_I2CM_RESET_SHIFT, 31
.define     I2CM1_CONTROL_I2CM_RESET_MASK,  0x80000000

.define REG_I2CM1_TIMING_CONFIG0, 0xf00604
.define     I2CM1_TIMING_CONFIG0_SCL_LOW_PER_SHIFT, 0
.define     I2CM1_TIMING_CONFIG0_SCL_LOW_PER_MASK,  0xff
.define     I2CM1_TIMING_CONFIG0_SCL_HIGH_PER_SHIFT, 8
.define     I2CM1_TIMING_CONFIG0_SCL_HIGH_PER_MASK,  0xff00
.define     I2CM1_TIMING_CONFIG0_SCL_STOP_PER_SHIFT, 16
.define     I2CM1_TIMING_CONFIG0_SCL_STOP_PER_MASK,  0xff0000
.define     I2CM1_TIMING_CONFIG0_SCL_START_PER_SHIFT, 24
.define     I2CM1_TIMING_CONFIG0_SCL_START_PER_MASK,  0xff000000

.define REG_I2CM1_TIMING_CONFIG1, 0xf00608
.define     I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD_SHIFT, 0
.define     I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD_MASK,  0xff
.define     I2CM1_TIMING_CONFIG1_SDA_LOW_TIME_SHIFT, 8
.define     I2CM1_TIMING_CONFIG1_SDA_LOW_TIME_MASK,  0xff00
.define     I2CM1_TIMING_CONFIG1_SCL_RISE_TIME_SHIFT, 16
.define     I2CM1_TIMING_CONFIG1_SCL_RISE_TIME_MASK,  0xff0000
.define     I2CM1_TIMING_CONFIG1_RESERVED_31_24_SHIFT, 24
.define     I2CM1_TIMING_CONFIG1_RESERVED_31_24_MASK,  0xff000000

.define REG_I2CM1_CONFIG, 0xf0060c
.define     I2CM1_CONFIG_SDA_LOW_DET_DIS_SHIFT, 0
.define     I2CM1_CONFIG_SDA_LOW_DET_DIS_MASK,  0x1
.define     I2CM1_CONFIG_CK_STRETCH_EN_SHIFT, 1
.define     I2CM1_CONFIG_CK_STRETCH_EN_MASK,  0x2
.define     I2CM1_CONFIG_RESERVED_7_2_SHIFT, 2
.define     I2CM1_CONFIG_RESERVED_7_2_MASK,  0xfc
.define     I2CM1_CONFIG_I2CM_EN_SHIFT, 8
.define     I2CM1_CONFIG_I2CM_EN_MASK,  0x100
.define     I2CM1_CONFIG_RESERVED_31_9_SHIFT, 9
.define     I2CM1_CONFIG_RESERVED_31_9_MASK,  0xfffffe00

.define REG_I2CM1_STATUS, 0xf00610
.define     I2CM1_STATUS_BUSY_SHIFT, 0
.define     I2CM1_STATUS_BUSY_MASK,  0x1
.define     I2CM1_STATUS_CK_STRETCH_STAT_SHIFT, 1
.define     I2CM1_STATUS_CK_STRETCH_STAT_MASK,  0x2
.define     I2CM1_STATUS_NAK_SENT_SHIFT, 2
.define     I2CM1_STATUS_NAK_SENT_MASK,  0x4
.define     I2CM1_STATUS_DATA_TRANS_STAT_SHIFT, 3
.define     I2CM1_STATUS_DATA_TRANS_STAT_MASK,  0x8
.define     I2CM1_STATUS_STOP_STAT_SHIFT, 4
.define     I2CM1_STATUS_STOP_STAT_MASK,  0x10
.define     I2CM1_STATUS_NAK_RCVD_SHIFT, 5
.define     I2CM1_STATUS_NAK_RCVD_MASK,  0x20
.define     I2CM1_STATUS_REG_ADDR_SENT_SHIFT, 6
.define     I2CM1_STATUS_REG_ADDR_SENT_MASK,  0x40
.define     I2CM1_STATUS_RD_SEQ_ACT_SHIFT, 7
.define     I2CM1_STATUS_RD_SEQ_ACT_MASK,  0x80
.define     I2CM1_STATUS_WR_SEQ_ACT_SHIFT, 8
.define     I2CM1_STATUS_WR_SEQ_ACT_MASK,  0x100
.define     I2CM1_STATUS_START_SENT_SHIFT, 9
.define     I2CM1_STATUS_START_SENT_MASK,  0x200
.define     I2CM1_STATUS_SDA_LOW_STAT_SHIFT, 10
.define     I2CM1_STATUS_SDA_LOW_STAT_MASK,  0x400
.define     I2CM1_STATUS_START_TRANS_STAT_SHIFT, 11
.define     I2CM1_STATUS_START_TRANS_STAT_MASK,  0x800
.define     I2CM1_STATUS_STOP_TRANS_STAT_SHIFT, 12
.define     I2CM1_STATUS_STOP_TRANS_STAT_MASK,  0x1000
.define     I2CM1_STATUS_PAUSE_STAT_SHIFT, 13
.define     I2CM1_STATUS_PAUSE_STAT_MASK,  0x2000
.define     I2CM1_STATUS_RESERVED_15_14_SHIFT, 14
.define     I2CM1_STATUS_RESERVED_15_14_MASK,  0xc000
.define     I2CM1_STATUS_TRANS_LENGTH_SHIFT, 16
.define     I2CM1_STATUS_TRANS_LENGTH_MASK,  0x1f0000
.define     I2CM1_STATUS_RESERVED_31_21_SHIFT, 21
.define     I2CM1_STATUS_RESERVED_31_21_MASK,  0xffe00000

.define REG_I2CM1_TX_BUFFER0, 0xf00614 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM1_TX_BUFFER1, 0xf00618 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM1_TX_BUFFER2, 0xf0061c ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM1_TX_BUFFER3, 0xf00620 ; This register holds the data to be transmitted during write transactions.

.define REG_I2CM1_RX_BUFFER0, 0xf00624 ; This register holds the data received during read transactions.

.define REG_I2CM1_RX_BUFFER1, 0xf00628 ; This register holds the data received during read transactions.

.define REG_I2CM1_RX_BUFFER2, 0xf0062c ; This register holds the data received during read transactions.

.define REG_I2CM1_RX_BUFFER3, 0xf00630 ; This register holds the data received during read transactions.


.endif /* !I2CM_S */
