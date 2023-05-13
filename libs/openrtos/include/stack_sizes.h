#include <FreeRTOS.h>
#include <task.h>
#include <interrupts.h>

/* Idle Thread */
extern StackType_t xIdleTaskStack[];
extern UInt32      uIdleTaskStackSize;

/* Host Thread */
extern StackType_t xHostStack[];
extern UInt32      uHostStackSize;

/* Sensor Framework Thread */
extern StackType_t xSensorStack[];
extern UInt32      uSensorStackSize;

/* Virtual Sensor Threads */
#define VIRTUAL_SENSOR_TASKS    ((PRIORITY_LOWEST - PRIORITY_2) + 1)  // 3 tasks; priorities 5, 6, 7
extern StackType_t* xVirtualStacks[VIRTUAL_SENSOR_TASKS];
extern UInt32       uVirtualStacksSize[VIRTUAL_SENSOR_TASKS];

