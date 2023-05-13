/*%---------------------------------------------------------------------
%   Date:     2014/12/03
%   Revision: 0
%   Author:   Timo Giesselmann
%   Supervisor: Dominik Geisler
%   Department: BST/ESA1
%---------------------------------------------------------------------
% This is the initialization function for activity recognition which
% shall be used within a BHy SW Patch.
% It overwrites the dummy function in ROM and uses the allocated
% memory areas instead of null pointers
%---------------------------------------------------------------------
*/
#if (ROM_VERSION == ROM_7183_DI01)

#ifdef MATLAB_MEX_FILE
#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#   include "stdint.h"
#endif
#include "activity_recognition_wrapper.h"
#include "activity_recognition_init.h"

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif

#include "features_cal.h"

signed short accnx_ar_init[150];
signed short accny_ar_init[150];
signed short accnz_ar_init[150];
short data_tilt_ar_init[150];
signed short ph_acc_ar_init[150];
signed short macc_ar_init[150];

extern volatile short has_the_function_activity_recognition_hwfun_init_been_patched;

void activity_recognition_hwfun_init(void)
{
    // Initialize the array pointers to allocated memory areas
    accn.x =    accnx_ar_init;
    accn.y =    accny_ar_init;
    accn.z =    accnz_ar_init;
    data_tilt = data_tilt_ar_init;
    ph_acc =    ph_acc_ar_init;
    macc =      macc_ar_init;

    has_the_function_activity_recognition_hwfun_init_been_patched = 1; // This is the patched init function

    activity_recognition_hwfun_reset();
#ifdef MATLAB_MEX_FILE
    mexPrintf("patched activity_recognition_hwfun_init function finished\n");
#endif
}

#endif
