////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/types.h
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

#ifndef INT_TYPES_H
#define INT_TYPES_H

#if defined(_MSC_VER)
    #include <stdint.h>
#endif

#define MAX_UINT16                  65536               /**< @brief Maximum value in a 16 but unsigned integer */
#define MIN_UINT16                  0                   /**< @brief Minimum value in a 16 but unsigned integer */
#define MAX_SINT16                  32767               /**< @brief Maximum value in a 16 but signed integer */
#define MIN_SINT16                  -32768              /**< @brief Minimum value in a 16 but signed integer */

#define KB_PER_SECTORS (4)

#ifndef TRUE
#define TRUE            ((bool)1)       /**< True */
#endif /* !TRUE */

#ifndef FALSE
#define FALSE           ((bool)0)       /**< False */
#endif /* !FALSE */

#ifndef NULL
#define NULL            ((void*)0)  /**< Null pointer */
#endif /* NULL */

#if defined(_FRAMEWORK)
#define IN_TEXT
#else
#define IN_TEXT         __attribute__ ((section (".text")))     /**< @brief Ensure that the specific symbol resides in the text section. */
#endif

#if defined(_FRAMEWORK)
#define RECLAIM_TEXT
#else
#define RECLAIM_TEXT    __attribute__ ((section (".reclaim_text")))  /**< @brief Ensure that the specific symbol is reclaimable. */
#endif

#if defined(_FRAMEWORK)
#define RAM_TEXT
#else
#define RAM_TEXT        __attribute__ ((section (".ram_text")))  /**< @brief Ensure that the function is always in ram, even if running flash firmware. */
#endif

#if defined(_FRAMEWORK)
#define SPECIAL_DATA
#else
#define SPECIAL_DATA    __attribute__ ((section (".special_data")))  /**< @brief Ensure that the specific symbol resides in the special data section. */
#endif

#if defined(_FRAMEWORK)
#define RECLAIM_DATA
#else
#define RECLAIM_DATA    __attribute__ ((section (".reclaim_data")))  /**< @brief Ensure that the specific symbol is reclaimable. */
#endif

#if defined(_FRAMEWORK)
#define NOJLI
#else
#define NOJLI         __attribute__ ((section (".nojli")))  /**< @brief Special sections for the check_jli script to verify that no jli calls are located here. */
#endif

#if defined(NO_JLI_CALLS)      // allow NO_JLI_CALLS to be specified to disable calling functions through the JLI table.
#define JLI
#else
#define JLI           __attribute__((jli_always))   /**< @brief The specified function should be called through a JLI entry. Can be overridden by defining NO_JLI_CALLS before including any headers */
#endif

#if defined(_MSC_VER)
    #define DEPRECATED(__x__) __pragma(deprecated(__x__))
#else
    #if defined(__clang__) || (__GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 4)))
    #define DEPRECATED(__x__)   __attribute__ ((deprecated(__x__)))
    #else
    #define DEPRECATED(__x__)   __attribute__ ((deprecated))
    #endif
#endif

#define UNUSED(__x__)      ((void)(__x__))                      /**< @brief Signal to the linter tool that the variable is explicitly unused. */

#define SATURATE(__max__, __x__, __min__) \
    (((__x__) > (__max__)) ? (__max__) : ((__x__) < (__min__) ? (__min__) : (__x__)))

#define MIN(__x__, __y__)   (((__x__) > (__y__)) ? (__y__) : (__x__))

#define MAX(__x__, __y__)   (((__x__) > (__y__)) ? (__x__) : (__y__))

#define ARRAY_ELEMENTS(__x__)                       ((sizeof(__x__)/sizeof(__x__[0])))
#define ELEMENT_OFFSET(__struct__, __elememnt__)    __builtin_offsetof(__struct__, __elememnt__)

#ifndef PREPACK
    #if defined(_MSC_VER)
    #define PACK
    #define PREPACK __pragma(pack(push, 1))
    #define MIDPACK
    #define POSTPACK __pragma(pack(pop))
    #else
    #define PREPACK
    #define MIDPACK __attribute__((packed))
    #define POSTPACK
    #endif
#endif

#if defined(_ARCVER)
#define SECTION(__x__) __attribute__ ((section (__x__)))
#else
#define SECTION(__x__)
#endif

#if defined(_ARCVER)
#define ALIGNED(x)   __attribute__((aligned((x))))
#else
#define ALIGNED(x)
#endif


#include <../7189/includes/types.h>


#endif /* INT_TYPES_H */
