/******************************************************************************
 **
 ** @file       linker_user.cmd
 **
 ** @project    7189
 **
 ** @brief      Initial linker script for user mode code in ram.
 **
 ** @classification  Confidential
 **
 ******************************************************************************
 **
 ******************************************************************************
 **
 ** @copyright Copyright (c) 2017, EM Microelectronic-US
 ** @cond
 **
 ** All rights reserved.
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are met:
 ** 1. Redistributions of source code must retain the above copyright notice,
 ** this list of conditions and the following disclaimer.
 ** 2. Redistributions in binary form must reproduce the above copyright notice,
 ** this list of conditions and the following disclaimer in the documentation
 ** and/or other materials provided with the distribution.
 **
 ******************************************************************************
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 ** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 ** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 ** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 ** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 ** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 ** INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 ** CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 ** POSSIBILITY OF SUCH DAMAGE.
 ** @endcond
 ******************************************************************************/

/* DO NOT REMOVE/MODIFY */
FIXED_ICCM_START = 1196032;
FIXED_DCCM_START = 10502144;
FW_HEADER_ADDR = 1291152;
CODE_RAM_ADDR  = 1291152 + 128;
DATA_RAM_ADDR  = 10561448;
KERNEL_FW_START = 1196032;

/* DO NOT REMOVE/MODIFY */
MEMORY {
    FW_HEADER     :   ORIGIN = FW_HEADER_ADDR LENGTH = 128
    CODE_RAM      :   ORIGIN = CODE_RAM_ADDR LENGTH = 150640 - 128
    DATA_RAM      :   ORIGIN = DATA_RAM_ADDR LENGTH = 186456
    JLI_REGION    :   ORIGIN = 0 LENGTH = 1
}

SECTIONS {
    /*
     * Ensure the linker throws an error in the event that we
     *  accidentally use the jli instruction.
     */
    /* DO NOT REMOVE/MODIFY */
    .jlitab ALIGN(4) : {
        *(.jlitab)
    } > JLI_REGION

    /**************************************************************************/
    /*********************** FW Header (in code ram) **************************/
    /**************************************************************************/
    /* DO NOT REMOVE/MODIFY */
    .fw_header FW_HEADER_ADDR : {
        *(.fw_header_orig*)
        *(.fw_header_offset*)
    } > FW_HEADER

    /* DO NOT REMOVE/MODIFY */
    .fw_header_copy ALIGN(4) : {
        _fw_header_copy_offset = . - CODE_RAM_ADDR + 4;
        *(.fw_header_copy*)
    } > CODE_RAM

    /**************************************************************************/
    /*********************** Code Ram *****************************************/
    /**************************************************************************/
    /* DO NOT REMOVE/MODIFY */
    .vectors CODE_RAM_ADDR + SIZEOF(.fw_header_copy) : {
        *(.vectors*)
    } > CODE_RAM

    /* DO NOT REMOVE */
    .text ALIGN(4) : {
        *(.nojli*)
        *(.text*)
        /*
         * Add any new text sections here
         */
    } > CODE_RAM

    /* DO NOT REMOVE/MODIFY */
    .em7189_descriptor : { *(.em7189_descriptor*) } > CODE_RAM

    /* DO NOT REMOVE */
    .reclaim : {
        /* All Memory after here can be reclaimed */
        _ffreecoderam = .;
        *(.reclaim_data)
        *(.reclaim*)
        /*
         * Add any new reclaimable sections here.
         */
    } > CODE_RAM

    /* DO NOT REMOVE/MODIFY */
    .initdat_stub ALIGN(4): {
        _initdat = .;
    } > CODE_RAM

    /* DO NOT REMOVE/MODIFY */
    .freecoderam : {
        _efreecoderam = .;
    } > CODE_RAM

    /**************************************************************************/
    /*********************** Data Ram *****************************************/
    /**************************************************************************/
    /* Disabled until user mode needs a separate stack.
    _fstack = ALIGN(4);
    .stack ALIGN(8) SIZE(_STACKSIZE): {}
    _estack = _fstack + _STACKSIZE;
    */

    /* DO NOT REMOVE/MODIFY */
    .bss DATA_RAM_ADDR : {
        *(.bss*)
        _fbss = ADDR(.bss);
        _ebss = .;
    } > DATA_RAM

    .sdata ALIGN(4): {
        __SDATA_BEGIN__ = .;
        *(.sdata*)
    } > DATA_RAM

    .data ALIGN(4): {
        *(.data*)
        *(.test_group*)
    } > DATA_RAM

    .rodata ALIGN(4): {
        * (LIT)
        *(.rodata*)
    } > DATA_RAM

    /* Sensor Descriptors */
    /* DO NOT REMOVE/MODIFY */
    .phys_sensor_descriptors : {
        *(.phys_sensor_descriptors)
        _fphys_sensor_descriptors = ADDR(.phys_sensor_descriptors);
        _ephys_sensor_descriptors = .;
    } > DATA_RAM

    .virt_sensor_descriptors : {
        *(.virt_sensor_descriptors)
        _fvirt_sensor_descriptors = ADDR(.virt_sensor_descriptors);
        _evirt_sensor_descriptors = .;
    } > DATA_RAM

    .timer_sensor_descriptors : {
        *(.timer_sensor_descriptors)
        _ftimer_sensor_descriptors = ADDR(.timer_sensor_descriptors);
        _etimer_sensor_descriptors = .;
    } > DATA_RAM


    /*
     * List all possible hooks here.
     */
    .hook_PhysicalRate : {
        *(.hook*_PhysicalRate)
        _fhook_PhysicalRate = ADDR(.hook_PhysicalRate);
        _ehook_PhysicalRate = .;
    } > DATA_RAM

    .hook_TimerRate : {
        *(.hook*_TimerRate)
        _fhook_TimerRate = ADDR(.hook_TimerRate);
        _ehook_TimerRate = .;
    } > DATA_RAM

    .hook_PhysicalRateChanged : {
        *(.hook*_PhysicalRateChanged)
        _fhook_PhysicalRateChanged = ADDR(.hook_PhysicalRateChanged);
        _ehook_PhysicalRateChanged = .;
    } > DATA_RAM

    .hook_PhysicalRangeChanged : {
        *(.hook*_PhysicalRangeChanged)
        _fhook_PhysicalRangeChanged = ADDR(.hook_PhysicalRangeChanged);
        _ehook_PhysicalRangeChanged = .;
    } > DATA_RAM

    .hook_VirtualSensorsDetermined : {
        *(.hook*_VirtualSensorsDetermined)
        _fhook_VirtualSensorsDetermined = ADDR(.hook_VirtualSensorsDetermined);
        _ehook_VirtualSensorsDetermined = .;
    } > DATA_RAM

    .hook_OverrideMaxRate : {
        *(.hook*_OverrideMaxRate)
        _fhook_OverrideMaxRate = ADDR(.hook_OverrideMaxRate);
        _ehook_OverrideMaxRate = .;
    } > DATA_RAM

    .hook_updatePhysicalState : {
        *(.hook*_updatePhysicalState)
        _fhook_updatePhysicalState = ADDR(.hook_updatePhysicalState);
        _ehook_updatePhysicalState = .;
    } > DATA_RAM

    .hook_determinePowerState : {
        *(.hook*_determinePowerState)
        _fhook_determinePowerState = ADDR(.hook_determinePowerState);
        _ehook_determinePowerState = .;
    } > DATA_RAM

    .hook_ignoreStandby : {
        *(.hook*_ignoreStandby)
        _fhook_ignoreStandby = ADDR(.hook_ignoreStandby);
        _ehook_ignoreStandby = .;
    } > DATA_RAM

    .hook_initOnce : {
        *(.hook*_initOnce)
        _fhook_initOnce = ADDR(.hook_initOnce);
        _ehook_initOnce = .;
    } > DATA_RAM

    .hook_initialize : {
        *(.hook*_initialize)
        _fhook_initialize = ADDR(.hook_initialize);
        _ehook_initialize = .;
    } > DATA_RAM

    .hook_exitShutdown : {
        *(.hook*_exitShutdown)
        _fhook_exitShutdown = ADDR(.hook_exitShutdown);
        _ehook_exitShutdown = .;
    } > DATA_RAM

    .hook_teardown : {
        *(.hook*_teardown)
        _fhook_teardown = ADDR(.hook_teardown);
        _ehook_teardown = .;
    } > DATA_RAM

    .hook_HIFRead : {
        *(.hook*_HIFRead)
        _fhook_HIFRead = ADDR(.hook_HIFRead);
        _ehook_HIFRead = .;
    } > DATA_RAM

    .hook_HIFWrite : {
        *(.hook*_HIFWrite)
        _fhook_HIFWrite = ADDR(.hook_HIFWrite);
        _ehook_HIFWrite = .;
    } > DATA_RAM

    .hook_HIFComm : {
        *(.hook*_HIFComm)
        _fhook_HIFComm = ADDR(.hook_HIFComm);
        _ehook_HIFComm = .;
    } > DATA_RAM

    .hook_HIFErr : {
        *(.hook*_HIFErr)
        _fhook_HIFErr = ADDR(.hook_HIFErr);
        _ehook_HIFErr = .;
    } > DATA_RAM

    .hook_HIFCommand : {
        *(.hook*_HIFCommand)
        _fhook_HIFCommand = ADDR(.hook_HIFCommand);
        _ehook_HIFCommand = .;
    } > DATA_RAM


    /* DO NOT REMOVE/MODIFY */
    .freedataram : {
        _ffreedataram = .;
        _efreedataram = .;
    } > DATA_RAM
}

/* Determine the end of the current Code RAM bank. */
/* DO NOT REMOVE/MODIFY */
_usedcoderam_kernel = FW_HEADER_ADDR - FIXED_ICCM_START;
_usedcoderam_user   = _ffreecoderam - FW_HEADER_ADDR;
_usedcoderam        = _ffreecoderam - FIXED_ICCM_START;
_optionalcodebanks  = (_usedcoderam > INITIAL_BANK_SIZE) ? (_usedcoderam - INITIAL_BANK_SIZE) : 0;
_optionalcodebanks  = (_optionalcodebanks & (OPTIONAL_BANK_SIZE - 1)) ? ((_optionalcodebanks / OPTIONAL_BANK_SIZE) + 1) : (_optionalcodebanks / OPTIONAL_BANK_SIZE);
_poweredcoderam     = INITIAL_BANK_SIZE + (_optionalcodebanks * OPTIONAL_BANK_SIZE);
_efreecoderam       = FIXED_ICCM_START + _poweredcoderam;

/* Determine the end of the current Data RAM bank. */
/* DO NOT REMOVE/MODIFY */
_useddataram_kernel = DATA_RAM_ADDR - FIXED_DCCM_START;
_useddataram_user   = _efreedataram - DATA_RAM_ADDR;
_useddataram        = _efreedataram - FIXED_DCCM_START;
_optionaldatabanks  = (_useddataram > INITIAL_BANK_SIZE) ? (_useddataram - INITIAL_BANK_SIZE) : 0;
_optionaldatabanks  = (_optionaldatabanks & (OPTIONAL_BANK_SIZE - 1)) ? ((_optionaldatabanks / OPTIONAL_BANK_SIZE) + 1) : (_optionaldatabanks / OPTIONAL_BANK_SIZE);
_powereddataram     = INITIAL_BANK_SIZE + (_optionaldatabanks * OPTIONAL_BANK_SIZE);
_efreedataram       = FIXED_DCCM_START + _powereddataram;
_totaloptionalbanks = _optionalcodebanks + _optionaldatabanks;
