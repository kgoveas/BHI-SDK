////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/types.h
///
/// @project    EM7189
///
/// @brief      Standard data types.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

#ifndef TYPES_H
#define TYPES_H

typedef signed char         SInt8;              /**< Signed 8 bit integer. */
typedef unsigned char       UInt8;              /**< Unsigned 8 bit integer. */
typedef signed short        SInt16;             /**< Signed 16 bit integer. */
typedef unsigned short      UInt16;             /**< Unsigned 16 bit integer. */
typedef signed int          SInt32;             /**< Signed 32 bit integer. */
typedef unsigned int        UInt32;             /**< Unsigned 32 bit integer. */
typedef signed long long    SInt64;             /**< Signed 64 bit integer. */
typedef unsigned long long  UInt64;             /**< Unsigned 64 bit integer. */

#ifndef __cplusplus
typedef _Bool               bool;               /**< Boolean. */
#endif /* __cplusplus */

typedef UInt64              SystemTime_t;       /**< Data type used to hold the system time. */
typedef UInt32              TimestampTime_t;    /**< Data type used to hold the hardware timestamp. */

#ifdef _ARCVER          /* ARC, use native prototype */
#define PTR(__type__)                       __type__
#elif _FRAMEWORK
#define PTR(__type__)                       __type__
#else
#define PTR(__type__)                       UInt32
#endif

#ifdef _ARCVER
#define FUNC_PTR(__name__, __proto__)        __proto__        /* ARC, use native prototype */
#elif _FRAMEWORK
#define FUNC_PTR(__name__, __proto__)        __proto__        /* use native prototype */
#define _Inline
#define    _Interrupt
#else
#define FUNC_PTR(__name__, __proto__)        UInt32 __name__     /* Non arc, ensure pointers are 32bit */
#define _Inline                                /* Non-arc */
#endif

#ifdef _ARCVER
#define ALWAYS_INLINE   inline __attribute__ ((always_inline))
#else
#define ALWAYS_INLINE   inline
#endif

#define NEVER_INLINE   __attribute__ ((noinline))


/* Don't let GCC see this, probably better ways to do this **/

extern UInt8 VERSION_bosch_rom[];
static UInt16 ALWAYS_INLINE getExpectedROMVersion(void)
{
    // Grab the rom version we linked against.
    union {
        void* linker_version;
        UInt32 version;
    } cast;

    cast.linker_version = (void*)VERSION_bosch_rom;
    return (UInt16)cast.version;
}


/**
 * @fn UInt32 cast_float2int(float input);
 *
 * @brief Treat the floating point value as an integer without conversion.
 *
 * @param    input    The floating point value to use
 *
 * @returns The binary representation of the floating point input
 */
static ALWAYS_INLINE UInt32 cast_float2int(float input)
{
    typedef union {
        float asFloat;
        UInt32 asUInt32;
    } cast;

    cast caster;
    caster.asFloat = input;
    return caster.asUInt32;
}


/**
 * @fn float cast_int2float(UInt32 input);
 *
 * @brief Treat the integer value as a floating point number without conversion.
 *
 * @param    input    The integer value to use
 *
 * @returns The float representation of the binary input
 */
static ALWAYS_INLINE float cast_int2float(UInt32 input)
{
    typedef union {
        float asFloat;
        UInt32 asUInt32;
    } cast;

    cast caster;
    caster.asUInt32 = input;
    return caster.asFloat;
}

#endif /* TYPES_H */
