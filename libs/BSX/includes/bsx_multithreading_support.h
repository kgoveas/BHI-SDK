 /*
 * $license$
 */

/*!
 * @defgroup bsx_multithreading_support
 * @brief
 * @{*/


#ifndef BSX_MULTITHREADING_SUPPORT_H_
#define BSX_MULTITHREADING_SUPPORT_H_


/********************************************************/
/* header includes */
#include "bsx_datatypes.h"


/********************************************************/
/* function prototype declarations */


/* forward definition of the documentation */
/*! @def BSX_MT_TRIGGER_THREAD(x)
 * @brief Place holder macro for a call-back function to push a frame to a thread message queue
 *
 * @param[in] x Identifier of the thread
 */

/*! @def BSX_ENTER_CRITICAL_SECTION()
 * @brief Place holder macro for call-back function to enter a critical code section
 */

/*! @def BSX_LEAVE_CRITICAL_SECTION()
 * @brief Place holder macro for call-back function to leave a critical code section
 */


#ifdef ENABLE_MULTITHREADING_CALIBRATION

/*! @brief Enter critical code section by disabling all interrupts
 *
 * Callback function to integrate BSX with an operating system to avoid race conditions when accessing
 * common memory among threads, especially memory accesses from different threads.
 */
void bsx_enter_critical_section(void);

/*! @brief Leave critical code section by re-enabling all previous interrupts
 *
 * Callback function to integrate BSX with an operating system to avoid race conditions when accessing
 * common memory among threads, especially memory accesses from different threads.
 */
void bsx_leave_critical_section(void);

/*! @brief Signals to the OS that a thread shall be executed
 *
 * Callback function to integrate with an operating system to signal that an extra thread needs to be run
 *
 * @param[in] x Identifier of the thread
 */
void bsx_mt_trigger_thread(uint8_t x);


#define BSX_ENTER_CRITICAL_SECTION()    bsx_enter_critical_section()
#define BSX_LEAVE_CRITICAL_SECTION()    bsx_leave_critical_section()

#define BSX_MT_TRIGGER_THREAD(x)        bsx_mt_trigger_thread(x)

#else

#define BSX_ENTER_CRITICAL_SECTION()
#define BSX_LEAVE_CRITICAL_SECTION()

#define BSX_MT_TRIGGER_THREAD(x)

#endif


#endif /* BSX_MULTITHREADING_SUPPORT_H_ */

/** @}*/


