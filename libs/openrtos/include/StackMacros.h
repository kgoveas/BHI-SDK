/*
    OpenRTOS V9.0.0  Copyright (C) 2015 Real Time Engineers Ltd.

    This file is part of the OpenRTOS product.

    OpenRTOS is distributed exclusively by WITTENSTEIN high integrity systems,
    and is subject to the terms of the License granted to your organization,
    including its warranties and limitations on use and distribution. It cannot be
    copied or reproduced in any way except as permitted by the License.

    Licenses authorize use by processor, compiler, business unit, and product.

    WITTENSTEIN high integrity systems is a trading name of WITTENSTEIN
    aerospace & simulation ltd, Registered Office: Brown's Court, Long Ashton
    Business Park, Yanley Lane, Long Ashton, Bristol, BS41 9LB, UK.
    Tel: +44 (0) 1275 395 600, fax: +44 (0) 1275 393 630.
    E-mail: info@HighIntegritySystems.com

    http://www.HighIntegritySystems.com
*/

#ifndef STACK_MACROS_H
#define STACK_MACROS_H

/*
 * Call the stack overflow hook function if the stack of the task being swapped
 * out is currently overflowed, or looks like it might have overflowed in the
 * past.
 *
 * Setting configCHECK_FOR_STACK_OVERFLOW to 1 will cause the macro to check
 * the current stack state only - comparing the current top of stack value to
 * the stack limit.  Setting configCHECK_FOR_STACK_OVERFLOW to greater than 1
 * will also cause the last few stack bytes to be checked to ensure the value
 * to which the bytes were set when the task was created have not been
 * overwritten.  Note this second test does not guarantee that an overflowed
 * stack will always be recognised.
 */

/*-----------------------------------------------------------*/

#if( ( configCHECK_FOR_STACK_OVERFLOW == 1 ) && ( portSTACK_GROWTH < 0 ) )

    /* Only the current stack state is to be checked. */
    #define taskCHECK_FOR_STACK_OVERFLOW()                                                              \
    {                                                                                                   \
        PROC.Gp7.r8[0] = pxCurrentTCB->pcTaskName[0];                                                   \
        PROC.Gp7.r8[1] = pxCurrentTCB->pcTaskName[1];                                                   \
        PROC.Gp7.r16[1] = (UInt16)(pxCurrentTCB->pxTopOfStack - pxCurrentTCB->pxStack);                 \
        /* Is the currently saved stack pointer within the stack limit? */                              \
        if( pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack )                                       \
        {                                                                                               \
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB, pxCurrentTCB->pcTaskName );   \
        }                                                                                               \
    }

#endif /* configCHECK_FOR_STACK_OVERFLOW == 1 */
/*-----------------------------------------------------------*/

#if( ( configCHECK_FOR_STACK_OVERFLOW == 1 ) && ( portSTACK_GROWTH > 0 ) )

    /* Only the current stack state is to be checked. */
    #define taskCHECK_FOR_STACK_OVERFLOW()                                                              \
    {                                                                                                   \
                                                                                                        \
        /* Is the currently saved stack pointer within the stack limit? */                              \
        if( pxCurrentTCB->pxTopOfStack >= pxCurrentTCB->pxEndOfStack )                                  \
        {                                                                                               \
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB, pxCurrentTCB->pcTaskName );   \
        }                                                                                               \
    }

#endif /* configCHECK_FOR_STACK_OVERFLOW == 1 */
/*-----------------------------------------------------------*/

#if( ( configCHECK_FOR_STACK_OVERFLOW > 1 ) && ( portSTACK_GROWTH < 0 ) )

#define taskCHECK_FOR_STACK_OVERFLOW()                                                                  \
    {                                                                                                   \
        const uint32_t * const pulStack = ( uint32_t * ) pxCurrentTCB->pxStack;                         \
        const uint32_t ulCheckValue = ( uint32_t ) 0xa5a5a5a5;                                          \
        static UInt16 min_free_stack = (UInt16)0xffff;                                                  \
        UInt16 free_stack = (UInt16)(pxCurrentTCB->pxTopOfStack - pxCurrentTCB->pxStack);               \
        if (free_stack < min_free_stack)                                                                \
        {                                                                                               \
            PROC.Gp7.r8[0] = pxCurrentTCB->pcTaskName[0];                                               \
            PROC.Gp7.r8[1] = pxCurrentTCB->pcTaskName[1];                                               \
            PROC.Gp7.r16[1] = min_free_stack = free_stack;                                              \
        }                                                                                               \
        if( ( pulStack[ 0 ] != ulCheckValue ) ||                                                        \
            ( pulStack[ 1 ] != ulCheckValue ) ||                                                        \
            ( pulStack[ 2 ] != ulCheckValue ) ||                                                        \
            ( pulStack[ 3 ] != ulCheckValue ) )                                                         \
        {                                                                                               \
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB, pxCurrentTCB->pcTaskName );   \
        }                                                                                               \
    }

#endif /* #if( configCHECK_FOR_STACK_OVERFLOW > 1 ) */
/*-----------------------------------------------------------*/

#if( ( configCHECK_FOR_STACK_OVERFLOW > 1 ) && ( portSTACK_GROWTH > 0 ) )

    #define taskCHECK_FOR_STACK_OVERFLOW()                                                                                              \
    {                                                                                                                                   \
    int8_t *pcEndOfStack = ( int8_t * ) pxCurrentTCB->pxEndOfStack;                                                                     \
    static const uint8_t ucExpectedStackBytes[] = { tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE };   \
                                                                                                                                        \
                                                                                                                                        \
        pcEndOfStack -= sizeof( ucExpectedStackBytes );                                                                                 \
                                                                                                                                        \
        /* Has the extremity of the task stack ever been written over? */                                                               \
        if( memcmp( ( void * ) pcEndOfStack, ( void * ) ucExpectedStackBytes, sizeof( ucExpectedStackBytes ) ) != 0 )                   \
        {                                                                                                                               \
            vApplicationStackOverflowHook( ( TaskHandle_t ) pxCurrentTCB, pxCurrentTCB->pcTaskName );                                   \
        }                                                                                                                               \
    }

#endif /* #if( configCHECK_FOR_STACK_OVERFLOW > 1 ) */
/*-----------------------------------------------------------*/

/* Remove stack overflow macro if not being used. */
#ifndef taskCHECK_FOR_STACK_OVERFLOW
    #define taskCHECK_FOR_STACK_OVERFLOW()
#endif



#endif /* STACK_MACROS_H */
