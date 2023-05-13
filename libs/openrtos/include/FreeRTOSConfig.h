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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H


/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                        1
#define configUSE_IDLE_HOOK                         1
#define configUSE_TICK_HOOK                         0
#define configCPU_CLOCK_HZ                          ( ( unsigned long ) (getSYSOSCFrequency() * 1000u) )
#define configTICK_RATE_HZ                          ( ( portTickType ) 500 )
#define configMAX_PRIORITIES                        ( 8 )
#define configMINIMAL_STACK_SIZE                    ( ( unsigned short ) 512 ) // 2KB min
#define configTOTAL_HEAP_SIZE                       ( ( size_t ) ( 0 ) )
#define configMAX_TASK_NAME_LEN                     ( 16 )
#define configUSE_TRACE_FACILITY                    0
#define configUSE_CUSTOM_TRACE_FACILITY             0   // added by EMUS
#define configUSE_16_BIT_TICKS                      0
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1

#define configSUPPORT_DYNAMIC_ALLOCATION            0
#define configCHECK_FOR_STACK_OVERFLOW              0
#define configUSE_MALLOC_FAILED_HOOK                0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1
#define configSUPPORT_STATIC_ALLOCATION             1
#define configUSE_TICKLESS_IDLE                     1


/* Set max syscall interrupt priority to around half available number of priorities,
 * e.g. for 16, use 8 - interrupts from 0 thru 7 inclusive (highest priority = 0)
 * can still interrupt the kernel when it's in a critical section and during context
 * switches. */
#ifdef TARGET_EM_STARTER_KIT
    #define configNUMBER_INTERRUPT_PRIORITIES       2
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY    1
#else
    #define configNUMBER_INTERRUPT_PRIORITIES       8
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY    4
#endif

#define configLOWEST_INTERRUPT_PRIORITY             ( configNUMBER_INTERRUPT_PRIORITIES - 1 )

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                       0
#define configMAX_CO_ROUTINE_PRIORITIES             ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskCleanUpResources               0
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xQueueGetMutexHolder                1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1


// tracing

//                                 conflicts
//#define LP_PIN 4  // MCSB2   <-- TMG49033_INT
#define TICK_PIN 6  // MCSB4   <-- Camera_Vsync
#define TIN_PIN  8  // QSPICLK <-- flash
#define TNOT_PIN 12 // QSPID2  <-- flash
#define LP_PIN   13 // QSPID3  <-- flash

/* Used to perform any necessary initialisation - for example, open a file
into which trace is to be written. */
#if 0

#define traceTASK_CREATE( pxNewTCB ) { \
    enableGPIOOutput(LP_PIN); \
    enableGPIOOutput(TICK_PIN); \
    enableGPIOOutput(TNOT_PIN); \
    enableGPIOOutput(TIN_PIN); \
    UNUSED(pxNewTCB); }

/* Called immediately before entering tickless idle. */
#define traceLOW_POWER_IDLE_BEGIN() setGPIOHigh(LP_PIN)

/* Called when returning to the Idle task after a tickless idle. */
#define traceLOW_POWER_IDLE_END() setGPIOLow(LP_PIN)

/* Called after a task has been selected to run.  pxCurrentTCB holds a pointer
to the task control block of the selected task. */
#define traceTASK_SWITCHED_IN() setGPIOHigh(TIN_PIN)

/* Called before a task has been selected to run.  pxCurrentTCB holds a pointer
to the task control block of the task being switched out. */
#define traceTASK_SWITCHED_OUT() setGPIOLow(TIN_PIN)

/* Called during the tick interrupt. */
#define traceTASK_INCREMENT_TICK( xTickCount ) toggleGPIO(TICK_PIN)

#define traceTASK_NOTIFY_WAIT() setGPIOHigh(TNOT_PIN)
#define traceTASK_NOTIFY() setGPIOLow(TNOT_PIN)
#define traceTASK_NOTIFY_FROM_ISR() setGPIOLow(TNOT_PIN)
#endif

#endif /* FREERTOS_CONFIG_H */
