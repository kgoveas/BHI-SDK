;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; @file       common/7189/includes/peripherals.s
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

.ifndef PERIPHERALS_S
.define PERIPHERALS_S, 1


.define REG_CHP_TRIM0, 0xf00100
.define     CHP_TRIM0_REG_TRIM_LO_SHIFT, 0
.define     CHP_TRIM0_REG_TRIM_LO_MASK,  0xf
.define     CHP_TRIM0_RESERVED_7_4_SHIFT, 4
.define     CHP_TRIM0_RESERVED_7_4_MASK,  0xf0
.define     CHP_TRIM0_REG_TRIM_HI_SHIFT, 8
.define     CHP_TRIM0_REG_TRIM_HI_MASK,  0xf00
.define     CHP_TRIM0_RESERVED_15_12_SHIFT, 12
.define     CHP_TRIM0_RESERVED_15_12_MASK,  0xf000
.define     CHP_TRIM0_POR_TRIM_LO_SHIFT, 16
.define     CHP_TRIM0_POR_TRIM_LO_MASK,  0xf0000
.define     CHP_TRIM0_RESERVED_23_20_SHIFT, 20
.define     CHP_TRIM0_RESERVED_23_20_MASK,  0xf00000
.define     CHP_TRIM0_POR_TRIM_HI_SHIFT, 24
.define     CHP_TRIM0_POR_TRIM_HI_MASK,  0xf000000
.define     CHP_TRIM0_RESERVED_31_28_SHIFT, 28
.define     CHP_TRIM0_RESERVED_31_28_MASK,  0xf0000000

.define REG_CHP_TRIM1, 0xf00104
.define     CHP_TRIM1_SOSC_TRIM_LO_SHIFT, 0
.define     CHP_TRIM1_SOSC_TRIM_LO_MASK,  0xff
.define     CHP_TRIM1_SOSC_TRIM_HI_SHIFT, 8
.define     CHP_TRIM1_SOSC_TRIM_HI_MASK,  0xff00
.define     CHP_TRIM1_TOSC_TRIM_SHIFT, 16
.define     CHP_TRIM1_TOSC_TRIM_MASK,  0xff0000
.define     CHP_TRIM1_GEN_CTRL_SHIFT, 24
.define     CHP_TRIM1_GEN_CTRL_MASK,  0xf000000
.define     CHP_TRIM1_RESERVED_31_28_SHIFT, 28
.define     CHP_TRIM1_RESERVED_31_28_MASK,  0xf0000000

.define REG_CHP_POWERCONFIG, 0xf00108
.define     CHP_POWERCONFIG_RUN_LVL_SHIFT, 0
.define     CHP_POWERCONFIG_RUN_LVL_MASK,  0x3
.define     CHP_POWERCONFIG_RUN_LVL_20MHZ_AT_1_1V, 0x0
.define     CHP_POWERCONFIG_RUN_LVL_20MHZ_AT_0_86V, 0x1
.define     CHP_POWERCONFIG_RUN_LVL_50MHZ_AT_1_1V, 0x2
.define     CHP_POWERCONFIG_RUN_LVL_OTP_PROGRAMMING, 0x3

.define     CHP_POWERCONFIG_RESERVED_7_2_SHIFT, 2
.define     CHP_POWERCONFIG_RESERVED_7_2_MASK,  0xfc
.define     CHP_POWERCONFIG_RAM_PWR_SHIFT, 8
.define     CHP_POWERCONFIG_RAM_PWR_MASK,  0x7f00
.define     CHP_POWERCONFIG_RESERVED_15_15_SHIFT, 15
.define     CHP_POWERCONFIG_RESERVED_15_15_MASK,  0x8000
.define     CHP_POWERCONFIG_RAM_RST_N_SHIFT, 16
.define     CHP_POWERCONFIG_RAM_RST_N_MASK,  0x7f0000
.define     CHP_POWERCONFIG_RESERVED_31_23_SHIFT, 23
.define     CHP_POWERCONFIG_RESERVED_31_23_MASK,  0xff800000

.define REG_CHP_MEMCONFIG, 0xf0010c
.define     CHP_MEMCONFIG_FLASH_SIZE_SHIFT, 0
.define     CHP_MEMCONFIG_FLASH_SIZE_MASK,  0x3
.define     CHP_MEMCONFIG_FLASH_SIZE_1M_EFLASH, 0x0
.define     CHP_MEMCONFIG_FLASH_SIZE_2M_EFLASH, 0x1
.define     CHP_MEMCONFIG_FLASH_SIZE_4M_EFLASH, 0x2
.define     CHP_MEMCONFIG_FLASH_SIZE_8M_EFLASH, 0x3

.define     CHP_MEMCONFIG_RESERVED_3_2_SHIFT, 2
.define     CHP_MEMCONFIG_RESERVED_3_2_MASK,  0xc
.define     CHP_MEMCONFIG_FLASH_CFG_SHIFT, 4
.define     CHP_MEMCONFIG_FLASH_CFG_MASK,  0x10
.define     CHP_MEMCONFIG_RESERVED_7_5_SHIFT, 5
.define     CHP_MEMCONFIG_RESERVED_7_5_MASK,  0xe0
.define     CHP_MEMCONFIG_DCCM_NUM_SHIFT, 8
.define     CHP_MEMCONFIG_DCCM_NUM_MASK,  0x700
.define     CHP_MEMCONFIG_RESERVED_31_11_SHIFT, 11
.define     CHP_MEMCONFIG_RESERVED_31_11_MASK,  0xfffff800

.define REG_CHP_CONTROL, 0xf00110
.define     CHP_CONTROL_TOSC_EN_SHIFT, 0
.define     CHP_CONTROL_TOSC_EN_MASK,  0x1
.define     CHP_CONTROL_QSPI_SPEED_SHIFT, 1
.define     CHP_CONTROL_QSPI_SPEED_MASK,  0x2
.define     CHP_CONTROL_RESERVED_7_2_SHIFT, 2
.define     CHP_CONTROL_RESERVED_7_2_MASK,  0xfc
.define     CHP_CONTROL_RAM_BIST_EN_SHIFT, 8
.define     CHP_CONTROL_RAM_BIST_EN_MASK,  0x100
.define     CHP_CONTROL_RAM_BIST_LCK_SHIFT, 9
.define     CHP_CONTROL_RAM_BIST_LCK_MASK,  0x200
.define     CHP_CONTROL_RESERVED_15_10_SHIFT, 10
.define     CHP_CONTROL_RESERVED_15_10_MASK,  0xfc00
.define     CHP_CONTROL_OTP_PG_DIS_SHIFT, 16
.define     CHP_CONTROL_OTP_PG_DIS_MASK,  0x10000
.define     CHP_CONTROL_OTP_PG_LCK_SHIFT, 17
.define     CHP_CONTROL_OTP_PG_LCK_MASK,  0x20000
.define     CHP_CONTROL_RESERVED_23_18_SHIFT, 18
.define     CHP_CONTROL_RESERVED_23_18_MASK,  0xfc0000
.define     CHP_CONTROL_OTP_ACC_DIS_SHIFT, 24
.define     CHP_CONTROL_OTP_ACC_DIS_MASK,  0x1000000
.define     CHP_CONTROL_OTP_ACC_LCK_SHIFT, 25
.define     CHP_CONTROL_OTP_ACC_LCK_MASK,  0x2000000
.define     CHP_CONTROL_RESERVED_31_26_SHIFT, 26
.define     CHP_CONTROL_RESERVED_31_26_MASK,  0xfc000000

.define REG_CHP_DIAGNOSTIC, 0xf00114
.define     CHP_DIAGNOSTIC_PM_STATE_SHIFT, 0
.define     CHP_DIAGNOSTIC_PM_STATE_MASK,  0xf
.define     CHP_DIAGNOSTIC_PM_STATE_PWRUP_1, 0x0
.define     CHP_DIAGNOSTIC_PM_STATE_PWRUP_2, 0x1
.define     CHP_DIAGNOSTIC_PM_STATE_ACTV_1, 0x2
.define     CHP_DIAGNOSTIC_PM_STATE_ACTV_2, 0x3
.define     CHP_DIAGNOSTIC_PM_STATE_LSLP_1, 0x4
.define     CHP_DIAGNOSTIC_PM_STATE_LSLP_2, 0x5
.define     CHP_DIAGNOSTIC_PM_STATE_RAMP, 0x6
.define     CHP_DIAGNOSTIC_PM_STATE_SLP, 0x8
.define     CHP_DIAGNOSTIC_PM_STATE_DSLP, 0x9
.define     CHP_DIAGNOSTIC_PM_STATE_TEST_MODE, 0xf

.define     CHP_DIAGNOSTIC_ROM_ACC_ERR_SHIFT, 4
.define     CHP_DIAGNOSTIC_ROM_ACC_ERR_MASK,  0x10
.define     CHP_DIAGNOSTIC_ICCM_ACC_ERR_SHIFT, 5
.define     CHP_DIAGNOSTIC_ICCM_ACC_ERR_MASK,  0x20
.define     CHP_DIAGNOSTIC_DCCM_ACC_ERR_SHIFT, 6
.define     CHP_DIAGNOSTIC_DCCM_ACC_ERR_MASK,  0x40
.define     CHP_DIAGNOSTIC_RAM_BIST_ERR_SHIFT, 7
.define     CHP_DIAGNOSTIC_RAM_BIST_ERR_MASK,  0x80
.define     CHP_DIAGNOSTIC_RST_SRC_SHIFT, 8
.define     CHP_DIAGNOSTIC_RST_SRC_MASK,  0x700
.define     CHP_DIAGNOSTIC_RST_SRC_EXTERNAL_RESET, 0x1
.define     CHP_DIAGNOSTIC_RST_SRC_HOST_RESET, 0x2
.define     CHP_DIAGNOSTIC_RST_SRC_WATCHDOG_RESET, 0x4

.define     CHP_DIAGNOSTIC_RESERVED_15_11_SHIFT, 11
.define     CHP_DIAGNOSTIC_RESERVED_15_11_MASK,  0xf800
.define     CHP_DIAGNOSTIC_QSPI_ACC_ERR_SHIFT, 16
.define     CHP_DIAGNOSTIC_QSPI_ACC_ERR_MASK,  0x10000
.define     CHP_DIAGNOSTIC_RESERVED_31_17_SHIFT, 17
.define     CHP_DIAGNOSTIC_RESERVED_31_17_MASK,  0xfffe0000


.define REG_FCC_CONTROL, 0xf00190
.define     FCC_CONTROL_FCC_EN_SHIFT, 0
.define     FCC_CONTROL_FCC_EN_MASK,  0x1
.define     FCC_CONTROL_RESERVED_31_1_SHIFT, 1
.define     FCC_CONTROL_RESERVED_31_1_MASK,  0xfffffffe


.define REG_FPGA_SENSOR_CONFIG, 0xf002fc ; This bit-field shows the current value of the real time counter.
.define     FPGA_SENSOR_CONFIG_MAG_SHIFT, 0
.define     FPGA_SENSOR_CONFIG_MAG_MASK,  0x1
.define     FPGA_SENSOR_CONFIG_MAG_BMM150, 0x0
.define     FPGA_SENSOR_CONFIG_MAG_AKM09915, 0x1

.define     FPGA_SENSOR_CONFIG_ACCEL_SHIFT, 1
.define     FPGA_SENSOR_CONFIG_ACCEL_MASK,  0x2
.define     FPGA_SENSOR_CONFIG_ACCEL_BMI160, 0x0
.define     FPGA_SENSOR_CONFIG_ACCEL_BMA2XY, 0x1



.define REG_ITMR_CONTROL, 0xf001a0
.define     ITMR_CONTROL_ITMR_EN_SHIFT, 0
.define     ITMR_CONTROL_ITMR_EN_MASK,  0x1
.define     ITMR_CONTROL_ITMR_CLR_SHIFT, 1
.define     ITMR_CONTROL_ITMR_CLR_MASK,  0x2
.define     ITMR_CONTROL_RESERVED_31_2_SHIFT, 2
.define     ITMR_CONTROL_RESERVED_31_2_MASK,  0xfffffffc

.define REG_ITMR_LIMIT, 0xf001a4
.define     ITMR_LIMIT_ITMR_LIMIT_SHIFT, 0
.define     ITMR_LIMIT_ITMR_LIMIT_MASK,  0xffffffff

.define REG_ITMR_VALUE, 0xf001a8
.define     ITMR_VALUE_ITMR_VALUE_SHIFT, 0
.define     ITMR_VALUE_ITMR_VALUE_MASK,  0xffffffff


.define REG_OTP_CONTROL, 0xf00200
.define     OTP_CONTROL_OTP_VDD_EN_Q_SHIFT, 0
.define     OTP_CONTROL_OTP_VDD_EN_Q_MASK,  0x1
.define     OTP_CONTROL_OTP_VDDQ_EN_Q_SHIFT, 1
.define     OTP_CONTROL_OTP_VDDQ_EN_Q_MASK,  0x2
.define     OTP_CONTROL_OTP_PGENB_Q_SHIFT, 2
.define     OTP_CONTROL_OTP_PGENB_Q_MASK,  0x4
.define     OTP_CONTROL_OTP_CSB_Q_SHIFT, 3
.define     OTP_CONTROL_OTP_CSB_Q_MASK,  0x8
.define     OTP_CONTROL_OTP_LOAD_Q_SHIFT, 4
.define     OTP_CONTROL_OTP_LOAD_Q_MASK,  0x10
.define     OTP_CONTROL_RESERVED_15_5_SHIFT, 5
.define     OTP_CONTROL_RESERVED_15_5_MASK,  0xffe0
.define     OTP_CONTROL_OTP_ADDR_SHIFT, 16
.define     OTP_CONTROL_OTP_ADDR_MASK,  0x3ff0000
.define     OTP_CONTROL_RESERVED_31_26_SHIFT, 26
.define     OTP_CONTROL_RESERVED_31_26_MASK,  0xfc000000

.define REG_OTP_STROBE, 0xf00204
.define     OTP_STROBE_OTP_STROBE_Q_SHIFT, 0
.define     OTP_STROBE_OTP_STROBE_Q_MASK,  0x1
.define     OTP_STROBE_RESERVED_31_1_SHIFT, 1
.define     OTP_STROBE_RESERVED_31_1_MASK,  0xfffffffe

.define REG_OTP_DATA, 0xf00208
.define     OTP_DATA_OTP_RD_DATA_SHIFT, 0
.define     OTP_DATA_OTP_RD_DATA_MASK,  0xff
.define     OTP_DATA_RESERVED_15_8_SHIFT, 8
.define     OTP_DATA_RESERVED_15_8_MASK,  0xff00
.define     OTP_DATA_OTP_VDDQ_SW_ON_SYNC_SHIFT, 16
.define     OTP_DATA_OTP_VDDQ_SW_ON_SYNC_MASK,  0x10000
.define     OTP_DATA_RESERVED_31_17_SHIFT, 17
.define     OTP_DATA_RESERVED_31_17_MASK,  0xfffe0000


.define REG_PAD_CONTROL0, 0xf00240
.define     PAD_CONTROL0_RESERVED_7_0_SHIFT, 0
.define     PAD_CONTROL0_RESERVED_7_0_MASK,  0xff
.define     PAD_CONTROL0_PAD_MFP_FUNC_SHIFT, 8
.define     PAD_CONTROL0_PAD_MFP_FUNC_MASK,  0x7ff00
.define     PAD_CONTROL0_PAD_MFP_FUNC_GPIO, 0x0
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_CLK, 0x1
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_CSN, 0x2
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO0, 0x4
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO1, 0x8
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO2, 0x10
.define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO3, 0x20
.define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_CLK, 0x40
.define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_DIO, 0x80
.define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_DIN, 0x100
.define     PAD_CONTROL0_PAD_MFP_FUNC_SIF3_SCL, 0x200
.define     PAD_CONTROL0_PAD_MFP_FUNC_SIF3_SDA, 0x400

.define     PAD_CONTROL0_RESERVED_31_19_SHIFT, 19
.define     PAD_CONTROL0_RESERVED_31_19_MASK,  0xfff80000

.define REG_PAD_CONTROL1, 0xf00244
.define     PAD_CONTROL1_PAD_JTAG_EN_SHIFT, 0
.define     PAD_CONTROL1_PAD_JTAG_EN_MASK,  0x1
.define     PAD_CONTROL1_PAD_JTAG_LCK_SHIFT, 1
.define     PAD_CONTROL1_PAD_JTAG_LCK_MASK,  0x2
.define     PAD_CONTROL1_RESERVED_31_2_SHIFT, 2
.define     PAD_CONTROL1_RESERVED_31_2_MASK,  0xfffffffc

.define REG_PAD_CONTROL2, 0xf00248
.define     PAD_CONTROL2_PAD_HIF_DIS_SHIFT, 0
.define     PAD_CONTROL2_PAD_HIF_DIS_MASK,  0x1
.define     PAD_CONTROL2_PAD_HIF_LCK_SHIFT, 1
.define     PAD_CONTROL2_PAD_HIF_LCK_MASK,  0x2
.define     PAD_CONTROL2_RESERVED_31_2_SHIFT, 2
.define     PAD_CONTROL2_RESERVED_31_2_MASK,  0xfffffffc

.define REG_PAD_DRIVECONTROL, 0xf0024c
.define     PAD_DRIVECONTROL_MFPAD_DRIVE_SHIFT, 0
.define     PAD_DRIVECONTROL_MFPAD_DRIVE_MASK,  0x1ffffff
.define     PAD_DRIVECONTROL_SIF1CLK_DRIVE_SHIFT, 25
.define     PAD_DRIVECONTROL_SIF1CLK_DRIVE_MASK,  0x2000000
.define     PAD_DRIVECONTROL_SIF1CSN_DRIVE_SHIFT, 26
.define     PAD_DRIVECONTROL_SIF1CSN_DRIVE_MASK,  0x4000000
.define     PAD_DRIVECONTROL_SIF1DIO_DRIVE_SHIFT, 27
.define     PAD_DRIVECONTROL_SIF1DIO_DRIVE_MASK,  0x8000000
.define     PAD_DRIVECONTROL_RESERVED_31_28_SHIFT, 28
.define     PAD_DRIVECONTROL_RESERVED_31_28_MASK,  0xf0000000

.define REG_PAD_PULLENABLE, 0xf00250
.define     PAD_PULLENABLE_MFPAD_PEN_SHIFT, 0
.define     PAD_PULLENABLE_MFPAD_PEN_MASK,  0x1ffffff
.define     PAD_PULLENABLE_SIF1CLK_PEN_SHIFT, 25
.define     PAD_PULLENABLE_SIF1CLK_PEN_MASK,  0x2000000
.define     PAD_PULLENABLE_SIF1DIN_PEN_SHIFT, 26
.define     PAD_PULLENABLE_SIF1DIN_PEN_MASK,  0x4000000
.define     PAD_PULLENABLE_SIF1DIO_PEN_SHIFT, 27
.define     PAD_PULLENABLE_SIF1DIO_PEN_MASK,  0x8000000
.define     PAD_PULLENABLE_RESERVED_31_28_SHIFT, 28
.define     PAD_PULLENABLE_RESERVED_31_28_MASK,  0xf0000000

.define REG_PAD_GPIODOUT, 0xf00254
.define     PAD_GPIODOUT_GPIO_DOUT_SHIFT, 0
.define     PAD_GPIODOUT_GPIO_DOUT_MASK,  0x1ffffff
.define     PAD_GPIODOUT_SIF1CSN_ROUT_SHIFT, 25
.define     PAD_GPIODOUT_SIF1CSN_ROUT_MASK,  0x2000000
.define     PAD_GPIODOUT_RESERVED_31_26_SHIFT, 26
.define     PAD_GPIODOUT_RESERVED_31_26_MASK,  0xfc000000

.define REG_PAD_GPIOOUTD, 0xf00258
.define     PAD_GPIOOUTD_GPIO_CFG_OUT_SHIFT, 0
.define     PAD_GPIOOUTD_GPIO_CFG_OUT_MASK,  0x1ffffff
.define     PAD_GPIOOUTD_RESERVED_31_25_SHIFT, 25
.define     PAD_GPIOOUTD_RESERVED_31_25_MASK,  0xfe000000

.define REG_PAD_GPIOOUTZ, 0xf0025c
.define     PAD_GPIOOUTZ_GPIO_CFG_HIZ_SHIFT, 0
.define     PAD_GPIOOUTZ_GPIO_CFG_HIZ_MASK,  0x1ffffff
.define     PAD_GPIOOUTZ_RESERVED_31_25_SHIFT, 25
.define     PAD_GPIOOUTZ_RESERVED_31_25_MASK,  0xfe000000

.define REG_PAD_GPIOIN, 0xf00260
.define     PAD_GPIOIN_GPIO_CFG_IN_SHIFT, 0
.define     PAD_GPIOIN_GPIO_CFG_IN_MASK,  0x1ffffff
.define     PAD_GPIOIN_RESERVED_31_25_SHIFT, 25
.define     PAD_GPIOIN_RESERVED_31_25_MASK,  0xfe000000

.define REG_PAD_GPIOINOUT, 0xf00264
.define     PAD_GPIOINOUT_GPIO_CFG_IO_SHIFT, 0
.define     PAD_GPIOINOUT_GPIO_CFG_IO_MASK,  0x1ffffff
.define     PAD_GPIOINOUT_RESERVED_31_25_SHIFT, 25
.define     PAD_GPIOINOUT_RESERVED_31_25_MASK,  0xfe000000

.define REG_PAD_GPIOSETTING0, 0xf00268
.define     PAD_GPIOSETTING0_MFPAD_IE_0_SHIFT, 0
.define     PAD_GPIOSETTING0_MFPAD_IE_0_MASK,  0x1
.define     PAD_GPIOSETTING0_MFPAD_OE_0_SHIFT, 1
.define     PAD_GPIOSETTING0_MFPAD_OE_0_MASK,  0x2
.define     PAD_GPIOSETTING0_MFPAD_IE_1_SHIFT, 2
.define     PAD_GPIOSETTING0_MFPAD_IE_1_MASK,  0x4
.define     PAD_GPIOSETTING0_MFPAD_OE_1_SHIFT, 3
.define     PAD_GPIOSETTING0_MFPAD_OE_1_MASK,  0x8
.define     PAD_GPIOSETTING0_MFPAD_IE_2_SHIFT, 4
.define     PAD_GPIOSETTING0_MFPAD_IE_2_MASK,  0x10
.define     PAD_GPIOSETTING0_MFPAD_OE_2_SHIFT, 5
.define     PAD_GPIOSETTING0_MFPAD_OE_2_MASK,  0x20
.define     PAD_GPIOSETTING0_MFPAD_IE_3_SHIFT, 6
.define     PAD_GPIOSETTING0_MFPAD_IE_3_MASK,  0x40
.define     PAD_GPIOSETTING0_MFPAD_OE_3_SHIFT, 7
.define     PAD_GPIOSETTING0_MFPAD_OE_3_MASK,  0x80
.define     PAD_GPIOSETTING0_MFPAD_IE_4_SHIFT, 8
.define     PAD_GPIOSETTING0_MFPAD_IE_4_MASK,  0x100
.define     PAD_GPIOSETTING0_MFPAD_OE_4_SHIFT, 9
.define     PAD_GPIOSETTING0_MFPAD_OE_4_MASK,  0x200
.define     PAD_GPIOSETTING0_MFPAD_IE_5_SHIFT, 10
.define     PAD_GPIOSETTING0_MFPAD_IE_5_MASK,  0x400
.define     PAD_GPIOSETTING0_MFPAD_OE_5_SHIFT, 11
.define     PAD_GPIOSETTING0_MFPAD_OE_5_MASK,  0x800
.define     PAD_GPIOSETTING0_MFPAD_IE_6_SHIFT, 12
.define     PAD_GPIOSETTING0_MFPAD_IE_6_MASK,  0x1000
.define     PAD_GPIOSETTING0_MFPAD_OE_6_SHIFT, 13
.define     PAD_GPIOSETTING0_MFPAD_OE_6_MASK,  0x2000
.define     PAD_GPIOSETTING0_MFPAD_IE_7_SHIFT, 14
.define     PAD_GPIOSETTING0_MFPAD_IE_7_MASK,  0x4000
.define     PAD_GPIOSETTING0_MFPAD_OE_7_SHIFT, 15
.define     PAD_GPIOSETTING0_MFPAD_OE_7_MASK,  0x8000
.define     PAD_GPIOSETTING0_MFPAD_IE_8_SHIFT, 16
.define     PAD_GPIOSETTING0_MFPAD_IE_8_MASK,  0x10000
.define     PAD_GPIOSETTING0_MFPAD_OE_8_SHIFT, 17
.define     PAD_GPIOSETTING0_MFPAD_OE_8_MASK,  0x20000
.define     PAD_GPIOSETTING0_MFPAD_IE_9_SHIFT, 18
.define     PAD_GPIOSETTING0_MFPAD_IE_9_MASK,  0x40000
.define     PAD_GPIOSETTING0_MFPAD_OE_9_SHIFT, 19
.define     PAD_GPIOSETTING0_MFPAD_OE_9_MASK,  0x80000
.define     PAD_GPIOSETTING0_MFPAD_IE_10_SHIFT, 20
.define     PAD_GPIOSETTING0_MFPAD_IE_10_MASK,  0x100000
.define     PAD_GPIOSETTING0_MFPAD_OE_10_SHIFT, 21
.define     PAD_GPIOSETTING0_MFPAD_OE_10_MASK,  0x200000
.define     PAD_GPIOSETTING0_MFPAD_IE_11_SHIFT, 22
.define     PAD_GPIOSETTING0_MFPAD_IE_11_MASK,  0x400000
.define     PAD_GPIOSETTING0_MFPAD_OE_11_SHIFT, 23
.define     PAD_GPIOSETTING0_MFPAD_OE_11_MASK,  0x800000
.define     PAD_GPIOSETTING0_MFPAD_IE_12_SHIFT, 24
.define     PAD_GPIOSETTING0_MFPAD_IE_12_MASK,  0x1000000
.define     PAD_GPIOSETTING0_MFPAD_OE_12_SHIFT, 25
.define     PAD_GPIOSETTING0_MFPAD_OE_12_MASK,  0x2000000
.define     PAD_GPIOSETTING0_MFPAD_IE_13_SHIFT, 26
.define     PAD_GPIOSETTING0_MFPAD_IE_13_MASK,  0x4000000
.define     PAD_GPIOSETTING0_MFPAD_OE_13_SHIFT, 27
.define     PAD_GPIOSETTING0_MFPAD_OE_13_MASK,  0x8000000
.define     PAD_GPIOSETTING0_MFPAD_IE_14_SHIFT, 28
.define     PAD_GPIOSETTING0_MFPAD_IE_14_MASK,  0x10000000
.define     PAD_GPIOSETTING0_MFPAD_OE_14_SHIFT, 29
.define     PAD_GPIOSETTING0_MFPAD_OE_14_MASK,  0x20000000
.define     PAD_GPIOSETTING0_MFPAD_IE_15_SHIFT, 30
.define     PAD_GPIOSETTING0_MFPAD_IE_15_MASK,  0x40000000
.define     PAD_GPIOSETTING0_MFPAD_OE_15_SHIFT, 31
.define     PAD_GPIOSETTING0_MFPAD_OE_15_MASK,  0x80000000

.define REG_PAD_GPIOSETTING1, 0xf0026c
.define     PAD_GPIOSETTING1_MFPAD_IE_16_SHIFT, 0
.define     PAD_GPIOSETTING1_MFPAD_IE_16_MASK,  0x1
.define     PAD_GPIOSETTING1_MFPAD_OE_16_SHIFT, 1
.define     PAD_GPIOSETTING1_MFPAD_OE_16_MASK,  0x2
.define     PAD_GPIOSETTING1_MFPAD_IE_17_SHIFT, 2
.define     PAD_GPIOSETTING1_MFPAD_IE_17_MASK,  0x4
.define     PAD_GPIOSETTING1_MFPAD_OE_17_SHIFT, 3
.define     PAD_GPIOSETTING1_MFPAD_OE_17_MASK,  0x8
.define     PAD_GPIOSETTING1_MFPAD_IE_18_SHIFT, 4
.define     PAD_GPIOSETTING1_MFPAD_IE_18_MASK,  0x10
.define     PAD_GPIOSETTING1_MFPAD_OE_18_SHIFT, 5
.define     PAD_GPIOSETTING1_MFPAD_OE_18_MASK,  0x20
.define     PAD_GPIOSETTING1_MFPAD_IE_19_SHIFT, 6
.define     PAD_GPIOSETTING1_MFPAD_IE_19_MASK,  0x40
.define     PAD_GPIOSETTING1_MFPAD_OE_19_SHIFT, 7
.define     PAD_GPIOSETTING1_MFPAD_OE_19_MASK,  0x80
.define     PAD_GPIOSETTING1_MFPAD_IE_20_SHIFT, 8
.define     PAD_GPIOSETTING1_MFPAD_IE_20_MASK,  0x100
.define     PAD_GPIOSETTING1_MFPAD_OE_20_SHIFT, 9
.define     PAD_GPIOSETTING1_MFPAD_OE_20_MASK,  0x200
.define     PAD_GPIOSETTING1_MFPAD_IE_21_SHIFT, 10
.define     PAD_GPIOSETTING1_MFPAD_IE_21_MASK,  0x400
.define     PAD_GPIOSETTING1_MFPAD_OE_21_SHIFT, 11
.define     PAD_GPIOSETTING1_MFPAD_OE_21_MASK,  0x800
.define     PAD_GPIOSETTING1_MFPAD_IE_22_SHIFT, 12
.define     PAD_GPIOSETTING1_MFPAD_IE_22_MASK,  0x1000
.define     PAD_GPIOSETTING1_MFPAD_OE_22_SHIFT, 13
.define     PAD_GPIOSETTING1_MFPAD_OE_22_MASK,  0x2000
.define     PAD_GPIOSETTING1_MFPAD_IE_23_SHIFT, 14
.define     PAD_GPIOSETTING1_MFPAD_IE_23_MASK,  0x4000
.define     PAD_GPIOSETTING1_MFPAD_OE_23_SHIFT, 15
.define     PAD_GPIOSETTING1_MFPAD_OE_23_MASK,  0x8000
.define     PAD_GPIOSETTING1_MFPAD_IE_24_SHIFT, 16
.define     PAD_GPIOSETTING1_MFPAD_IE_24_MASK,  0x10000
.define     PAD_GPIOSETTING1_MFPAD_OE_24_SHIFT, 17
.define     PAD_GPIOSETTING1_MFPAD_OE_24_MASK,  0x20000
.define     PAD_GPIOSETTING1_RESERVED_31_18_SHIFT, 18
.define     PAD_GPIOSETTING1_RESERVED_31_18_MASK,  0xfffc0000

.define REG_PAD_GPIOVALUE, 0xf00270
.define     PAD_GPIOVALUE_GPIO_VALUE_SHIFT, 0
.define     PAD_GPIOVALUE_GPIO_VALUE_MASK,  0x1ffffff
.define     PAD_GPIOVALUE_RESERVED_31_25_SHIFT, 25
.define     PAD_GPIOVALUE_RESERVED_31_25_MASK,  0xfe000000

.define REG_PAD_GPIOADDCONFIG, 0xf00274
.define     PAD_GPIOADDCONFIG_RESERVED_0_0_SHIFT, 0
.define     PAD_GPIOADDCONFIG_RESERVED_0_0_MASK,  0x1
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_SHIFT, 1
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_MASK,  0xffe
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT9_GPIO12, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT6_GPIO9, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT4_GPIO6, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT10_GPIO13, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT8_GPIO11, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT1_GPIO3, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT2_GPIO4, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT3_GPIO5, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT5_GPIO8, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT7_GPIO10, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT11_GPIO16, 0x0
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT1_GPIO1, 0x1
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT2_GPIO5, 0x2
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT3_GPIO4, 0x4
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT4_GPIO14, 0x8
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT5_GPIO9, 0x10
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT6_GPIO8, 0x20
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT7_GPIO15, 0x40
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT8_GPIO17, 0x80
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT9_GPIO18, 0x100
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT10_GPIO19, 0x200
.define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT11_GPIO20, 0x400

.define     PAD_GPIOADDCONFIG_RESERVED_15_12_SHIFT, 12
.define     PAD_GPIOADDCONFIG_RESERVED_15_12_MASK,  0xf000
.define     PAD_GPIOADDCONFIG_GPIO_UTMR_CFG_SHIFT, 16
.define     PAD_GPIOADDCONFIG_GPIO_UTMR_CFG_MASK,  0xf0000
.define     PAD_GPIOADDCONFIG_RESERVED_23_20_SHIFT, 20
.define     PAD_GPIOADDCONFIG_RESERVED_23_20_MASK,  0xf00000
.define     PAD_GPIOADDCONFIG_PAD_SIF_CFG_SHIFT, 24
.define     PAD_GPIOADDCONFIG_PAD_SIF_CFG_MASK,  0x3000000
.define     PAD_GPIOADDCONFIG_RESERVED_31_26_SHIFT, 26
.define     PAD_GPIOADDCONFIG_RESERVED_31_26_MASK,  0xfc000000

.define REG_PAD_GPIOEVIRQPOLARITY, 0xf00278
.define     PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL_SHIFT, 0
.define     PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL_MASK,  0xfff
.define     PAD_GPIOEVIRQPOLARITY_RESERVED_31_12_SHIFT, 12
.define     PAD_GPIOEVIRQPOLARITY_RESERVED_31_12_MASK,  0xfffff000

.define REG_PAD_GPIOEVIRQTIME0, 0xf0027c
.define     PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0_SHIFT, 0
.define     PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME1, 0xf00280
.define     PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1_SHIFT, 0
.define     PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME2, 0xf00284
.define     PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2_SHIFT, 0
.define     PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME3, 0xf00288
.define     PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3_SHIFT, 0
.define     PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME4, 0xf0028c
.define     PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4_SHIFT, 0
.define     PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME5, 0xf00290
.define     PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5_SHIFT, 0
.define     PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME6, 0xf00294
.define     PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6_SHIFT, 0
.define     PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME7, 0xf00298
.define     PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7_SHIFT, 0
.define     PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME8, 0xf0029c
.define     PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8_SHIFT, 0
.define     PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME9, 0xf002a0
.define     PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9_SHIFT, 0
.define     PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME10, 0xf002a4
.define     PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10_SHIFT, 0
.define     PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10_MASK,  0xffffffff

.define REG_PAD_GPIOEVIRQTIME11, 0xf002a8
.define     PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11_SHIFT, 0
.define     PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11_MASK,  0xffffffff

.define REG_PAD_HOSTEVIRQTIME, 0xf002ac
.define     PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME_SHIFT, 0
.define     PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME_MASK,  0xffffffff


.define REG_PIRQ_IRQENSET, 0xf00120
.define     PIRQ_IRQENSET_IRQ_EN_SET_SHIFT, 0
.define     PIRQ_IRQENSET_IRQ_EN_SET_MASK,  0x7ffffff
.define     PIRQ_IRQENSET_RESERVED_31_27_SHIFT, 27
.define     PIRQ_IRQENSET_RESERVED_31_27_MASK,  0xf8000000

.define REG_PIRQ_IRQENCLR, 0xf00124
.define     PIRQ_IRQENCLR_IRQ_EN_CLR_SHIFT, 0
.define     PIRQ_IRQENCLR_IRQ_EN_CLR_MASK,  0x7ffffff
.define     PIRQ_IRQENCLR_RESERVED_31_27_SHIFT, 27
.define     PIRQ_IRQENCLR_RESERVED_31_27_MASK,  0xf8000000

.define REG_PIRQ_IRQENVALUE, 0xf00128
.define     PIRQ_IRQENVALUE_IRQ_EN_VAL_SHIFT, 0
.define     PIRQ_IRQENVALUE_IRQ_EN_VAL_MASK,  0x7ffffff
.define     PIRQ_IRQENVALUE_RESERVED_31_27_SHIFT, 27
.define     PIRQ_IRQENVALUE_RESERVED_31_27_MASK,  0xf8000000

.define REG_PIRQ_STATUSCLR, 0xf0012c
.define     PIRQ_STATUSCLR_IRQ_STAT_CLR_SHIFT, 0
.define     PIRQ_STATUSCLR_IRQ_STAT_CLR_MASK,  0x7ffffff
.define     PIRQ_STATUSCLR_RESERVED_31_27_SHIFT, 27
.define     PIRQ_STATUSCLR_RESERVED_31_27_MASK,  0xf8000000

.define REG_PIRQ_STATUSVALUE, 0xf00130
.define     PIRQ_STATUSVALUE_IRQ_STAT_VAL_SHIFT, 0
.define     PIRQ_STATUSVALUE_IRQ_STAT_VAL_MASK,  0x7ffffff
.define     PIRQ_STATUSVALUE_RESERVED_31_27_SHIFT, 27
.define     PIRQ_STATUSVALUE_RESERVED_31_27_MASK,  0xf8000000

.define REG_PIRQ_IRQMASKSET, 0xf00134
.define     PIRQ_IRQMASKSET_IRQ_MASK_SET_SHIFT, 0
.define     PIRQ_IRQMASKSET_IRQ_MASK_SET_MASK,  0x1ffffff
.define     PIRQ_IRQMASKSET_RESERVED_31_25_SHIFT, 25
.define     PIRQ_IRQMASKSET_RESERVED_31_25_MASK,  0xfe000000

.define REG_PIRQ_IRQMASKCLR, 0xf00138
.define     PIRQ_IRQMASKCLR_IRQ_MASK_CLR_SHIFT, 0
.define     PIRQ_IRQMASKCLR_IRQ_MASK_CLR_MASK,  0x1ffffff
.define     PIRQ_IRQMASKCLR_RESERVED_31_25_SHIFT, 25
.define     PIRQ_IRQMASKCLR_RESERVED_31_25_MASK,  0xfe000000

.define REG_PIRQ_IRQMASKVALUE, 0xf0013c
.define     PIRQ_IRQMASKVALUE_IRQ_MASK_VAL_SHIFT, 0
.define     PIRQ_IRQMASKVALUE_IRQ_MASK_VAL_MASK,  0x1ffffff
.define     PIRQ_IRQMASKVALUE_RESERVED_31_25_SHIFT, 25
.define     PIRQ_IRQMASKVALUE_RESERVED_31_25_MASK,  0xfe000000

.define REG_PIRQ_SWIRQSET, 0xf00140
.define     PIRQ_SWIRQSET_SW_INT_SET_SHIFT, 0
.define     PIRQ_SWIRQSET_SW_INT_SET_MASK,  0xf
.define     PIRQ_SWIRQSET_RESERVED_31_4_SHIFT, 4
.define     PIRQ_SWIRQSET_RESERVED_31_4_MASK,  0xfffffff0

.define REG_PIRQ_SWIRQCLR, 0xf00144
.define     PIRQ_SWIRQCLR_SW_INT_CLR_SHIFT, 0
.define     PIRQ_SWIRQCLR_SW_INT_CLR_MASK,  0xf
.define     PIRQ_SWIRQCLR_RESERVED_31_4_SHIFT, 4
.define     PIRQ_SWIRQCLR_RESERVED_31_4_MASK,  0xfffffff0

.define REG_PIRQ_SWIRQVALUE, 0xf00148
.define     PIRQ_SWIRQVALUE_SW_INT_SHIFT, 0
.define     PIRQ_SWIRQVALUE_SW_INT_MASK,  0xf
.define     PIRQ_SWIRQVALUE_RESERVED_31_4_SHIFT, 4
.define     PIRQ_SWIRQVALUE_RESERVED_31_4_MASK,  0xfffffff0

.define REG_PIRQ_HRWIRQENSET, 0xf0014c
.define     PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET_SHIFT, 0
.define     PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET_MASK,  0xffff
.define     PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET_SHIFT, 16
.define     PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET_MASK,  0xffff0000

.define REG_PIRQ_HRWIRQENCLR, 0xf00150
.define     PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR_SHIFT, 0
.define     PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR_MASK,  0xffff
.define     PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR_SHIFT, 16
.define     PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR_MASK,  0xffff0000

.define REG_PIRQ_HRWIRQENVALUE, 0xf00154
.define     PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL_SHIFT, 0
.define     PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL_MASK,  0xffff
.define     PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL_SHIFT, 16
.define     PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL_MASK,  0xffff0000

.define REG_PIRQ_HRWSTATUSCLR, 0xf00158
.define     PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR_SHIFT, 0
.define     PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR_MASK,  0xffff
.define     PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR_SHIFT, 16
.define     PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR_MASK,  0xffff0000

.define REG_PIRQ_HRWSTATUSVALUE, 0xf0015c
.define     PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL_SHIFT, 0
.define     PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL_MASK,  0xffff
.define     PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL_SHIFT, 16
.define     PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL_MASK,  0xffff0000


.define REG_QSPI_CONTROL0, 0xf00170
.define     QSPI_CONTROL0_QSPI_EN_SHIFT, 0
.define     QSPI_CONTROL0_QSPI_EN_MASK,  0x1
.define     QSPI_CONTROL0_QSPI_PROT_SHIFT, 1
.define     QSPI_CONTROL0_QSPI_PROT_MASK,  0xe
.define     QSPI_CONTROL0_QSPI_RW_SHIFT, 4
.define     QSPI_CONTROL0_QSPI_RW_MASK,  0x10
.define     QSPI_CONTROL0_QSPI_CMD_LNGTH_SHIFT, 5
.define     QSPI_CONTROL0_QSPI_CMD_LNGTH_MASK,  0x20
.define     QSPI_CONTROL0_QSPI_ADDR_LNGTH_SHIFT, 6
.define     QSPI_CONTROL0_QSPI_ADDR_LNGTH_MASK,  0xc0
.define     QSPI_CONTROL0_QSPI_CFG_LNGTH_SHIFT, 8
.define     QSPI_CONTROL0_QSPI_CFG_LNGTH_MASK,  0xf00
.define     QSPI_CONTROL0_QSPI_DMY_LNGTH_SHIFT, 12
.define     QSPI_CONTROL0_QSPI_DMY_LNGTH_MASK,  0xf000
.define     QSPI_CONTROL0_QSPI_DATA_LNGTH_SHIFT, 16
.define     QSPI_CONTROL0_QSPI_DATA_LNGTH_MASK,  0x1ff0000
.define     QSPI_CONTROL0_RESERVED_30_25_SHIFT, 25
.define     QSPI_CONTROL0_RESERVED_30_25_MASK,  0x7e000000
.define     QSPI_CONTROL0_QSPI_REQ_SHIFT, 31
.define     QSPI_CONTROL0_QSPI_REQ_MASK,  0x80000000

.define REG_QSPI_CONTROL1, 0xf00174
.define     QSPI_CONTROL1_QSPI_RESET_SHIFT, 0
.define     QSPI_CONTROL1_QSPI_RESET_MASK,  0x1
.define     QSPI_CONTROL1_RESERVED_15_1_SHIFT, 1
.define     QSPI_CONTROL1_RESERVED_15_1_MASK,  0xfffe
.define     QSPI_CONTROL1_QSPI_PINS_SHIFT, 16
.define     QSPI_CONTROL1_QSPI_PINS_MASK,  0x10000
.define     QSPI_CONTROL1_QSPI_OUT0_SHIFT, 17
.define     QSPI_CONTROL1_QSPI_OUT0_MASK,  0x20000
.define     QSPI_CONTROL1_QSPI_OUT1_SHIFT, 18
.define     QSPI_CONTROL1_QSPI_OUT1_MASK,  0x40000
.define     QSPI_CONTROL1_RESERVED_31_19_SHIFT, 19
.define     QSPI_CONTROL1_RESERVED_31_19_MASK,  0xfff80000

.define REG_QSPI_INSTRUCTION0, 0xf00178
.define     QSPI_INSTRUCTION0_QSPI_ADDR_SHIFT, 0
.define     QSPI_INSTRUCTION0_QSPI_ADDR_MASK,  0xffffff
.define     QSPI_INSTRUCTION0_QSPI_CMD_SHIFT, 24
.define     QSPI_INSTRUCTION0_QSPI_CMD_MASK,  0xff000000

.define REG_QSPI_INSTRUCTION1, 0xf0017c
.define     QSPI_INSTRUCTION1_QSPI_CFG_SHIFT, 0
.define     QSPI_INSTRUCTION1_QSPI_CFG_MASK,  0xff
.define     QSPI_INSTRUCTION1_RESERVED_31_8_SHIFT, 8
.define     QSPI_INSTRUCTION1_RESERVED_31_8_MASK,  0xffffff00

.define REG_QSPI_DATA, 0xf00180 ; This register holds read data received during a read instruction or programming data to be transmitted during a write instruction.
.define     QSPI_DATA_QSPI_RD_DATA_SHIFT, 0
.define     QSPI_DATA_QSPI_RD_DATA_MASK,  0xffffffff

.define REG_QSPI_STATUS, 0xf00184
.define     QSPI_STATUS_QSPI_BUSY_SHIFT, 0
.define     QSPI_STATUS_QSPI_BUSY_MASK,  0x1
.define     QSPI_STATUS_RESERVED_31_1_SHIFT, 1
.define     QSPI_STATUS_RESERVED_31_1_MASK,  0xfffffffe


.define REG_RTC_VALUE, 0xf001b0 ; This bit-field shows the current value of the real time counter.
.define     RTC_VALUE_RTC_VALUE_SHIFT, 0
.define     RTC_VALUE_RTC_VALUE_MASK,  0xffffffff


.define REG_UTMR_CONTROL, 0xf002c0
.define     UTMR_CONTROL_UTMR_EN_SHIFT, 0
.define     UTMR_CONTROL_UTMR_EN_MASK,  0x1
.define     UTMR_CONTROL_UTMR_IRQ_SEL_SHIFT, 1
.define     UTMR_CONTROL_UTMR_IRQ_SEL_MASK,  0x6
.define     UTMR_CONTROL_UTMR_CLR_SHIFT, 3
.define     UTMR_CONTROL_UTMR_CLR_MASK,  0x8
.define     UTMR_CONTROL_UTMR_CLK_SRC_SHIFT, 4
.define     UTMR_CONTROL_UTMR_CLK_SRC_MASK,  0x30
.define     UTMR_CONTROL_UTMR_CLK_DIV_SHIFT, 6
.define     UTMR_CONTROL_UTMR_CLK_DIV_MASK,  0xc0
.define     UTMR_CONTROL_UTMR_START_SEL_SHIFT, 8
.define     UTMR_CONTROL_UTMR_START_SEL_MASK,  0x100
.define     UTMR_CONTROL_UTMR_START_ACT_SHIFT, 9
.define     UTMR_CONTROL_UTMR_START_ACT_MASK,  0x200
.define     UTMR_CONTROL_UTMR_CAP_SEL_SHIFT, 10
.define     UTMR_CONTROL_UTMR_CAP_SEL_MASK,  0x400
.define     UTMR_CONTROL_RESERVED_13_11_SHIFT, 11
.define     UTMR_CONTROL_RESERVED_13_11_MASK,  0x3800
.define     UTMR_CONTROL_UTMR_CAP_POL_SHIFT, 14
.define     UTMR_CONTROL_UTMR_CAP_POL_MASK,  0xc000
.define     UTMR_CONTROL_UTMR_RSTRT_SEL_SHIFT, 16
.define     UTMR_CONTROL_UTMR_RSTRT_SEL_MASK,  0x10000
.define     UTMR_CONTROL_UTMR_LIM_ACT_SHIFT, 17
.define     UTMR_CONTROL_UTMR_LIM_ACT_MASK,  0x60000
.define     UTMR_CONTROL_UTMR_CMP_ACT_SHIFT, 19
.define     UTMR_CONTROL_UTMR_CMP_ACT_MASK,  0x180000
.define     UTMR_CONTROL_UTMR_OUT_EN_SHIFT, 21
.define     UTMR_CONTROL_UTMR_OUT_EN_MASK,  0x200000
.define     UTMR_CONTROL_RESERVED_29_22_SHIFT, 22
.define     UTMR_CONTROL_RESERVED_29_22_MASK,  0x3fc00000
.define     UTMR_CONTROL_UTMR_SW_START_SHIFT, 30
.define     UTMR_CONTROL_UTMR_SW_START_MASK,  0x40000000
.define     UTMR_CONTROL_UTMR_SW_CAP_SHIFT, 31
.define     UTMR_CONTROL_UTMR_SW_CAP_MASK,  0x80000000

.define REG_UTMR_LIMIT, 0xf002c4
.define     UTMR_LIMIT_UTMR_LIMIT_SHIFT, 0
.define     UTMR_LIMIT_UTMR_LIMIT_MASK,  0xffff
.define     UTMR_LIMIT_RESERVED_31_16_SHIFT, 16
.define     UTMR_LIMIT_RESERVED_31_16_MASK,  0xffff0000

.define REG_UTMR_COMPARE, 0xf002c8
.define     UTMR_COMPARE_UTMR_CMP_VAL_SHIFT, 0
.define     UTMR_COMPARE_UTMR_CMP_VAL_MASK,  0xffff
.define     UTMR_COMPARE_RESERVED_31_16_SHIFT, 16
.define     UTMR_COMPARE_RESERVED_31_16_MASK,  0xffff0000

.define REG_UTMR_CAPTURE, 0xf002cc
.define     UTMR_CAPTURE_UTMR_CAP_VAL_SHIFT, 0
.define     UTMR_CAPTURE_UTMR_CAP_VAL_MASK,  0xffff
.define     UTMR_CAPTURE_RESERVED_31_16_SHIFT, 16
.define     UTMR_CAPTURE_RESERVED_31_16_MASK,  0xffff0000

.define REG_UTMR_VALUE, 0xf002d0
.define     UTMR_VALUE_UTMR_VALUE_SHIFT, 0
.define     UTMR_VALUE_UTMR_VALUE_MASK,  0xffff
.define     UTMR_VALUE_RESERVED_31_16_SHIFT, 16
.define     UTMR_VALUE_RESERVED_31_16_MASK,  0xffff0000

.define REG_UTMR_STATUS, 0xf002d4
.define     UTMR_STATUS_UTMR_BUSY_SHIFT, 0
.define     UTMR_STATUS_UTMR_BUSY_MASK,  0x1
.define     UTMR_STATUS_RESERVED_31_1_SHIFT, 1
.define     UTMR_STATUS_RESERVED_31_1_MASK,  0xfffffffe


.endif /* !PERIPHERALS_S */
