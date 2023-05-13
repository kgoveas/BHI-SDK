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


#ifndef PORTMACRO_DI01_H
#define PORTMACRO_DI01_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* CPU options: architecture specific. */
#define portCPU_ADDRESS_BITS            ( 24 )      /* Width of CPU address bus. */
#define portMPU_TOTAL_REGIONS           ( 8 )       /* Number of regions provided by MPU. */
#define portMPU_VERSION                 ( 3 )       /* Version of MPU - 2 or 3 */

/* MPU choices                          */
#define portNUM_STATIC_LO_PRI_REGIONS   ( 2 )       /* Number of low priority regions reserved for application global use. */


/* Exclude various sections when #included by assembler file(s). */
#ifndef __ASSEMBLER_SRC__

/* EMUS: variables to override the default ranges from the ROM */
extern uint32_t *mpu_region_kernel_iccm_start;
extern uint32_t mpu_region_kernel_iccm_size;
extern uint32_t *mpu_region_priv_data_start;
extern uint32_t mpu_region_priv_data_size;

/* Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  unsigned portLONG
#define portBASE_TYPE   long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#ifndef NULL
    #ifdef __cplusplus
        #define NULL    0
    #else
        #define NULL    ( ( void * ) 0 )
    #endif
#endif


#if( configUSE_16_BIT_TICKS == 1 )
    typedef uint16_t TickType_t;
    #define portMAX_DELAY 0xffff
#else
    typedef uint32_t TickType_t;
    #define portMAX_DELAY 0xffffffffUL

    /* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
    not need to be guarded with a critical section. */
    #define portTICK_TYPE_IS_ATOMIC 1
#endif

#ifndef configSTATIC_RESTRICT_VARIANT
    #define configSTATIC_RESTRICT_VARIANT 0
#endif

#endif /* __ASSEMBLER_SRC__ */

/* Architecture specifics. */
#define portBYTE_ALIGNMENT              4
#define portSTACK_GROWTH                ( -1 )
#define portTICK_PERIOD_MS              ( ( portTickType ) 1000 / configTICK_RATE_HZ )
#define portCRITICAL_NESTING_IN_TCB     0
#define portCPU_MAX_ADDR                ( 0xFFFFFFFFUL >> ( 32 - portCPU_ADDRESS_BITS ) )

/* Constants for managing interrupt masking */
#define portIE_ENABLE_AND_SET           ( 0x10 ) /* OR with interrupt mask when used with seti instruction */
#define portEBITS_DISABLE_LOWER_IRQS    ( configMAX_SYSCALL_INTERRUPT_PRIORITY - 1 )
#define portEBITS_ENABLE_ALL_IRQS       ( 0x0F )
#define portSETI_MASK_LOWER_IRQS        ( portEBITS_DISABLE_LOWER_IRQS | portIE_ENABLE_AND_SET )
#define portSETI_ALLOW_ALL_IRQS         ( portEBITS_ENABLE_ALL_IRQS | portIE_ENABLE_AND_SET )

/* AUX registers used by portable layer (stand alone rather than including the regs file) */
#define portAUXREG_MPU_EN               ( 0x0409 )  /* MPU_ENABLE */
#define portAUXREG_MPU_RDB0             ( 0x0422 )  /* MPU_RDB0 */
#define portAUXREG_MPU_RDP0             ( 0x0423 )  /* MPU_RDP0 */
#define portAUXREG_SCRATCH_DATA0        ( 0x0418 )
#define portAUXREG_AUX_USER_SP          ( 0x0D )
#define portAUXREG_AUX_IRQ_ACT          ( 0x43 )
#define portAUX_IRQ_CONTROL             ( 0x0e )
#define portAUXREG_COUNT0               ( 0x21 )    /* REG_COUNT0 */
#define portAUXREG_CONTROL0             ( 0x22 )    /* REG_CONTROL0 */
#define portAUXREG_LIMIT0               ( 0x23 )    /* REG_LIMIT0 */
#define portAUXREG_AUX_IRQ_HINT         ( 0x201 )   /* AUX_IRQ_HINT  */
#define portAUXREG_IRQ_PRIORITY         ( 0x206 )   /* IRQ_PRIORITY  */
#define portAUXREG_IRQ_SELECT           ( 0x40b )   /* IRQ_INTERRUPT */
#define portAUXREG_IRQ_ENABLE           ( 0x40c )   /* IRQ_ENABLE    */
#define portAUXREG_IRQ_TRIGGER          ( 0x40d )   /* IRQ_TRIGGER   */
#define portAUXREG_IRQ_INTERRUPT        ( 0x40b )
#define portAUXREG_ECR                  ( 0x403 )


#if portMPU_VERSION == 2
    #define portMPU_MIN_REGION_SIZE_DESC    ( 10 )  /* MPU region descriptor SIZE, minimum value, corresponds to 2K bytes */
    #define portMPU_MIN_REGION_SIZE         ( 2048 )
    #define portMPU_REGION_ADDRESS_MASK     ( 0xFFFFF800UL ) /* place regions on 2k boundary */
#elif portMPU_VERSION == 3
    #define portMPU_MIN_REGION_SIZE_DESC    ( 4 )  /* MPU region descriptor SIZE, minimum value, corresponds to 32 bytes */
    #define portMPU_MIN_REGION_SIZE         ( 32 )
    #define portMPU_REGION_ADDRESS_MASK     ( 0xFFFFFFE0UL ) /* place regions on 32byte boundaries */
#else
    #error "Unknown MPU version"
#endif

#define portMPU_MAX_REGION_SIZE_DESC        ( 31 )  /* MPU region descriptor SIZE, maximum value, corresponds to 4GB. */


/* SVC (trap) macros. In-line assembler to do a software exception, "trap" style
 * service call. The "memory" keyword in the clobber list serves as a compiler
 * memory barrier. The SVCR(x,_rv) form returns a value in _rv. */
#define SVC(x) __asm__ __volatile__ (                                   \
                                        "   mov_s %%r0, %%sp   \n"      \
                                        "   trap_s %0        \n"        \
                                        :: "i"(x) : "memory", "r0"      \
                                    )

#define SVCR(x,_rv)  __asm__ __volatile__ (                             \
                                            "   mov_s %%r0, %%sp \n"        \
                                            "   trap_s %1      \n"      \
                                            "   mov_s %0, %%r0  \n"     \
                                            "   nop_s          \n"      \
                                            : "=r"(_rv) : "i"(x) : "memory", "r0" \
                                          )

/* SVC (trap) numbers for various services */
#define portSVC_RAISE_PRIVILEGE         0
#define portSVC_ENTER_CRITICAL          1
#define portSVC_EXIT_CRITICAL           2
#define portSVC_YIELD                   3
#define portSVC_SET_INTERRUPT_MASK      4
#define portSVC_CLEAR_INTERRUPT_MASK    5
#define portSVC_LOWER_PRIVILEGE         6
#define portSVC_IS_IN_ISR               7
#define portSVC_START_SCHEDULER         8
#define portSVC_SYSENTER                9
#define portSVC_SYSEXIT                 10

/* IRQs for "pendSV" and timer tick. */
#define portSYSTICK_TIMER_IRQ           ( 16 )
#define portPEND_IRQ                    ( 16 )

/* MPU specific constants. See also above near top of this file. */
#define portMIN_MPU_REGIONS             ( 5 )
#define portUSING_MPU_WRAPPERS          1
#define portMPU_PRIVILEGE_BITNUM        ( 7 ) /* Note - bit set = USER mode (unprivileged) */
#define portMPU_PRIVILEGE_BIT           ( 1 << portMPU_PRIVILEGE_BITNUM )
#define portPRIVILEGE_BIT               ( 1 << 7 )

/* Permission bits for MPU descriptor attributes. */
#define portMPU_REGION_USER_EXECUTE     ( 1 << 3 )
#define portMPU_REGION_USER_WRITE       ( 1 << 4 )
#define portMPU_REGION_USER_READ        ( 1 << 5 )
#define portMPU_REGION_KERNEL_EXECUTE   ( 1 << 6 )
#define portMPU_REGION_KERNEL_WRITE     ( 1 << 7 )
#define portMPU_REGION_KERNEL_READ      ( 1 << 8 )

/* shortcut macros for programming MPU registers. r=region num, b=base addr, p=attribs/size. */
#define portAUXREG_WRITE_MPU_REGION_DESC( r, b, p )                                     \
    do {                                                                                \
            portWRITE_AUX_REGISTER( ( b ), ( ( ( r ) * 2 ) + portAUXREG_MPU_RDB0 ) );   \
            portWRITE_AUX_REGISTER( ( p ), ( ( ( r ) * 2 ) + portAUXREG_MPU_RDP0 ) );   \
    } while( 0 )

/* Version of above that adds "valid" bit to base address. */
#define portAUXREG_WRITE_MPU_REGION_DESC_V( r, b, p )                                   \
    portAUXREG_WRITE_MPU_REGION_DESC( ( r ), ( ( ( uint32_t ) ( b ) ) | portMPU_REGION_VALID ), ( p ) )


/*
 * MPU has 8 or 16 regions. Regions 0 and 1 remain fixed, while each task
 * updates a number of regions from 2 (stack) upwards, depending on number
 * available and number set aside for application global use. Ideally,
 * keep the number of task configurable regions as small as possible, to
 * avoid slowing the context switch too much. Do this by increasing the
 * number portNUM_STATIC_LO_PRI_REGIONS.
 *
 * Region 0 is highest and region 15 is lowest priority.
 *
 * Note that the MINIMUM REGION SIZE is either 2048 bytes or 32 bytes,
 * depending on the MPU version. The minimum alignment of a region is its size
 * in bytes, 2048 or 32 bytes for v2 or v3 MPU, resp. The base address omits
 * unused bits - v2 MPUs use bits 31:11 for base addr, v3 MPUs use 31:5.
 *
 * Although ICCM may start elsewhere, the vector table, at least, may be
 * be mapped at address 0x0000_0000, so that region needs to have access
 * permitted by the MPU.
 *
 */

/* Lowest priority - global background region. Does not use an MPU region register pair. Allows code and data access by default, no user write. */
#define portDEFAULT_REGION_ATTRIBS          ( 0 ) // EMUS: default is no access; (portMPU_REGION_KERNEL_READ | portMPU_REGION_KERNEL_WRITE | portMPU_REGION_KERNEL_EXECUTE | portMPU_REGION_USER_READ | portMPU_REGION_USER_EXECUTE )

/* Highest priority region. Uses lowest numbered MPU region register pair. */
#define portKERNEL_CODE_REGION_ATTRIBS      ( portMPU_REGION_KERNEL_READ | portMPU_REGION_KERNEL_WRITE | portMPU_REGION_KERNEL_EXECUTE )
#define portKERNEL_DATA_REGION_ATTRIBS      ( portMPU_REGION_KERNEL_READ | portMPU_REGION_KERNEL_WRITE | portMPU_REGION_USER_READ )

#define portUSER_CODE_REGION_ATTRIBS        ( portMPU_REGION_KERNEL_READ | portMPU_REGION_KERNEL_WRITE | portMPU_REGION_KERNEL_EXECUTE | portMPU_REGION_USER_READ | portMPU_REGION_USER_EXECUTE )
#define portUSER_DATA_REGION_ATTRIBS        ( portMPU_REGION_KERNEL_READ | portMPU_REGION_KERNEL_WRITE | portMPU_REGION_USER_READ | portMPU_REGION_USER_WRITE )
#define portUSER_STACK_REGION_ATTRIBS       portUSER_DATA_REGION_ATTRIBS


#define portAPP_STATIC_HI_PRI_REGION        ( 0 ) /* May be used to provide app-specific fixed high priority region. */
#define portAPP_STATIC_REGION               ( 1 ) /* Unused for now */
#define portUSER_CODE_REGION                ( 2 )
#define portUSER_DATA_REGION                ( 3 )
#define portKERNEL_CODE_REGION              ( 4 )
#define portKERNEL_DATA_REGION              ( 5 ) /* 6 and 7 will be the first and last app static low priority regions */
#define portFIRST_CONFIGURABLE_REGION       ( 7 ) /* EMUS: Not used */
#define portLAST_CONFIGURABLE_REGION        ( 7 ) /* EMUS: Not used */


#if( portLAST_CONFIGURABLE_REGION < portFIRST_CONFIGURABLE_REGION )
    #error "No per-task MPU regions available. portNUM_STATIC_LO_PRI_REGIONS too high?"
#endif

/* app-specific fixed low priority regions. */
#if( ( portMPU_TOTAL_REGIONS > ( portMIN_MPU_REGIONS + portNUM_STATIC_LO_PRI_REGIONS ) ) && ( portNUM_STATIC_LO_PRI_REGIONS > 0 ) )
    #define portFIRST_APP_STATIC_LO_PRI_REGION  ( portMPU_TOTAL_REGIONS - ( portNUM_STATIC_LO_PRI_REGIONS ) )
    #define portLAST_APP_STATIC_LO_PRI_REGION   ( portMPU_TOTAL_REGIONS - 1 )
#else
    #define portFIRST_APP_STATIC_LO_PRI_REGION  ( 255 )    /* Wildcard value, any number > max possible MPU regions */
    #define portLAST_APP_STATIC_LO_PRI_REGION   ( 255 )
#endif

/* Background default region is in MPU_EN register, not an MPU region register pair */


#define portNUM_CONFIGURABLE_REGIONS      0 // ( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )
#define portTOTAL_NUM_REGIONS             0 // ( portNUM_CONFIGURABLE_REGIONS + 1 ) /* Plus one to make space for the stack region. */
//#if ((portTOTAL_NUM_REGIONS != 3) || (portLAST_CONFIGURABLE_REGION != 5))
//#error wrong value
//#endif
#define portENABLE_MPU                      ( 0x40000000 | portDEFAULT_REGION_ATTRIBS )
#define portDISABLE_MPU                     ( 0 )

#define portMPU_REGION_VALID                ( 1UL )
#define portMPU_REGION_INVALID              ( 0UL )



/*-----------------------------------------------------------*/
/* Exclude the following if this .h file was included from   */
/* an assembler source file...                               */
#ifndef __ASSEMBLER_SRC__

#if portMPU_TOTAL_REGIONS < portMIN_MPU_REGIONS
    #error "Not enough MPU regions, need at least 5"
#endif

typedef struct MPU_REGION_REGISTERS
{
    uint32_t ulRegionBaseAddress;
    uint32_t ulRegionAttribute;
} xMPU_REGION_REGISTERS;

typedef struct MPU_SETTINGS
{
    xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
} xMPU_SETTINGS;

/* Architecture-specific optimisations. */
#if ( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )

    /* Check the configuration. */
    #if ( configMAX_PRIORITIES > 31 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 31.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
    #endif

    /* Store/clear the ready priorities in a bit map. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )      ( ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )       ( ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) ) )

    /* Use NORM instruction to find first set bit (up to bit 30) */
    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    do { __asm__ __volatile__ ( "   norm %0, %1" : "=r"(uxTopPriority) : "r"(uxReadyPriorities) ); uxTopPriority = 30 - uxTopPriority; } while ( 0 )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
extern void vPortYieldFromISR( void ); /* Note that this function is in assembler, in portasm.S */
extern UBaseType_t xPortSetInterruptMaskFromISR( void );
extern void vPortClearInterruptMaskFromISR( UBaseType_t x );

#define portYIELD()                             SVC( portSVC_YIELD )
#define portYIELD_FROM_ISR( xSwitchRequired )   if( xSwitchRequired ) vPortYieldFromISR()

#define portLOWER_PRIVILEGE()                   SVC( portSVC_LOWER_PRIVILEGE )
#define portRAISE_PRIVILEGE()                   SVCR( portSVC_RAISE_PRIVILEGE )
#define portSWITCH_TO_USER_MODE()               portLOWER_PRIVILEGE()

#define vPortResetPrivilege( xWasRunningPrivileged )  \
    do                                                \
    {                                                 \
        if( pdFALSE == ( xWasRunningPrivileged ) )    \
        {                                             \
            portLOWER_PRIVILEGE();                    \
        }                                             \
    } while ( 0 )

/*-----------------------------------------------------------*/

/* The "memory" clobbers function as a compiler memory barrier. */
#define prvDISABLE_INTERRUPTS()                 __asm__ __volatile__ ( "    clri      " ::: "memory" )
#define prvENABLE_INTERRUPTS()                  __asm__ __volatile__ ( "    seti    0 " ::: "memory" )

#define portSTART_FIRST_TASK()                  SVC( portSVC_START_SCHEDULER )

#define portENTER_CRITICAL()                    SVC( portSVC_ENTER_CRITICAL )
#define portEXIT_CRITICAL()                     SVC( portSVC_EXIT_CRITICAL )

#define portDISABLE_INTERRUPTS()                SVC( portSVC_SET_INTERRUPT_MASK )
#define portENABLE_INTERRUPTS()                 SVC( portSVC_CLEAR_INTERRUPT_MASK )

#define portSET_INTERRUPT_MASK_FROM_ISR()                           \
    ( {                                                             \
        register unsigned int __ret;                                \
        __asm__ __volatile__ (                                      \
                "   clri    %0 \n"                                  \
                "   seti    %1   "                                  \
                : "=r" ( __ret )                                    \
                : "i" ( portSETI_MASK_LOWER_IRQS )                  \
                : "memory" );                                       \
        __ret;                                                      \
    } )

#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )  __asm__ __volatile__ ( "    seti    %0 " :: "ir"( x ) : "memory" )

#define portIS_IN_ISR()                                             \
    ( {                                                             \
        register unsigned int __ret;                                \
        SVCR( portSVC_IS_IN_ISR, __ret);                            \
        __ret;                                                      \
    } )


/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  It is not
necessary to have or use these macros in order to use this port, but they are
defined because the common demo files (which build with all the ports) need them
in order to build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )
/*-----------------------------------------------------------*/

/* Access to CPU registers and instructions. The conditionals below try to cater for
   various different build environments that use differing intriniscs. The newer
   Metaware compiler, ccac, predefines __GNUC__ as it's based on clang, and generally
   caters for GCC syntax. However, its ARC intrinsics are as per the previous Metaware
   compiler, mcc. Fortunately, it also predefines __CCAC__ so can be distinguished from
   GCC. */
#ifdef __GNUC__
    #ifdef __CCAC__
        #define portWRITE_AUX_REGISTER( v, r )  _sr( ( ( unsigned long ) ( v ) ), ( ( unsigned long ) ( r ) ) )
        #define portREAD_AUX_REGISTER( r )      _lr( ( ( unsigned long ) ( r ) ) )
        #define portREAD_CORE_GP_REGISTER()     _core_read( 26 )
        #define portNOP()                       _nop()
        #define portUSING_INTRINSICS            1
    #else
        #ifdef __arc__
            #define portWRITE_AUX_REGISTER( v, r )  __builtin_arc_sr( v, r )
            #define portREAD_AUX_REGISTER( r )      __builtin_arc_lr( r )
            #define portREAD_CORE_GP_REGISTER()     __builtin_arc_core_read( 26 )
            #define portNOP()                       __builtin_arc_nop()
            #define portUSING_INTRINSICS            1
        #endif /* __arc__ */
    #endif /* __CCAC__ */
#else
    #ifdef _ARC /* Metaware compiler uses different intrinsics. */
        #define portWRITE_AUX_REGISTER( v, r )  _sr( v, r )
        #define portREAD_AUX_REGISTER( r )      _lr( r )
        #define portREAD_CORE_GP_REGISTER()     _core_read( 26 )
        #define portNOP()                       _nop()
        #define portUSING_INTRINSICS            1
    #endif /* _ARC */
#endif /* __GNUC__ */

#ifndef portUSING_INTRINSICS
    #define  portWRITE_AUX_REGISTER( value, reg )       \
        __asm__ __volatile__ (                          \
                    " sr    %0, [ %1 ] "                \
                    :: "ir"( value ), "i" ( reg ) );

    #define  portREAD_AUX_REGISTER( reg )               \
        ( {                                             \
            unsigned int __ret;                         \
            __asm__ __volatile__ (                      \
                    " lr    %0, [ %1 ] "                \
                    : "=r"( __ret )                     \
                    : "i" ( reg ) );                    \
            __ret;                                      \
        } )

    #define  portREAD_CORE_GP_REGISTER()                \
        ( {                                             \
            unsigned int __ret;                         \
            __asm__ __volatile__ (                      \
                    " mov   %0, r26 "                   \
                    : "=r"( __ret ) );                  \
            __ret;                                      \
        } )

    #define portNOP()   __asm__ __volatile__ ( "    nop_s    \n" )
#endif /* not USING_INTRINSICS */


#if  configUSE_TICKLESS_IDLE != 0
    extern __attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );

    #define portSUPPRESS_TICKS_AND_SLEEP( x )       vPortSuppressTicksAndSleep( ( x ) )
#endif

/*
 * Return the smallest MPU region size that a given number of bytes will fit
 * into.  The region size is returned as the value that should be programmed
 * into the region attribute register for that region.
 */
// EMUS: moved to header and made public
extern uint32_t vPortGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes );

extern void vInitAppMPUSettings(uint8_t *iccm_end, uint8_t *dccm_end, uint32_t flash_size);

extern void registerSyscallTable(void* table, uint32_t num_syscalls);

extern void registerKernelStackForUser(uint32_t base, uint32_t top);
/*-----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLER_SRC__ */ /* end of section excluded from assembler sources */

#endif /* PORTMACRO_DI01_H */
