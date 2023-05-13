#ifndef LIBSOPENRTOSINCLUDEJLI_H
#define LIBSOPENRTOSINCLUDEJLI_H

#pragma jli_call_always(vSetWakeTimeCallback);
#pragma jli_call_always(vSleepTickless);
#pragma jli_call_always(ulGetExternalTime);
#pragma jli_call_always(vSetWakeTimeInterrupt);
#pragma jli_call_always(vInitAppMPUSettings);
#pragma jli_call_always(vPortFree);
#pragma jli_call_always(vPortEndScheduler);
#pragma jli_call_always(xPortRaisePrivilege);
#pragma jli_call_always(vPortGetMPURegionSizeSetting);
#pragma jli_call_always(pxPortInitialiseStack);
#pragma jli_call_always(prvSetupMPU);
#pragma jli_call_always(prvSetupTimerInterrupt);
#pragma jli_call_always(portDISABLE_IRQ);
#pragma jli_call_always(portENABLE_IRQ);
#pragma jli_call_always(portSET_IRQ_PRIORITY);
#pragma jli_call_always(vPortStoreTaskMPUSettings);
#pragma jli_call_always(xPortStartScheduler);
#pragma jli_call_always(vPortSuppressTicksAndSleep);

// #pragma jli_call_always(vPortYieldFromISR);

#endif /* !LIBS/OPENRTOS/INCLUDE/JLI_H */
