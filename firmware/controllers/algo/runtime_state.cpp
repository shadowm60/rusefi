/*
 * runtime_state.cpp
 *
 *  Created on: Jan 31, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */

#include "global.h"
#include "runtime_state.h"

// todo: revive implementation! we shall measure how far is actual execution timestamp from desired execution timestamp
uint32_t maxSchedulingPrecisionLoss = 0;


extern uint32_t hipLastExecutionCount;

extern uint32_t maxLockedDuration;
extern uint32_t maxEventCallbackDuration;
extern uint32_t triggerMaxDuration;
extern int perSecondIrqDuration;
extern int perSecondIrqCounter;

#if EFI_PROD_CODE
extern uint32_t maxPrecisionCallbackDuration;
#endif /* EFI_PROD_CODE  */

extern uint32_t maxSchedulingPrecisionLoss;
extern uint32_t *cyccnt;

void resetMaxValues() {
#if EFI_PROD_CODE || EFI_SIMULATOR
	maxEventCallbackDuration = triggerMaxDuration = 0;
#endif /* EFI_PROD_CODE || EFI_SIMULATOR */

	maxSchedulingPrecisionLoss = 0;

#if EFI_CLOCK_LOCKS
	maxLockedDuration = 0;
#endif /* EFI_CLOCK_LOCKS */

#if EFI_PROD_CODE
	maxPrecisionCallbackDuration = 0;
#endif /* EFI_PROD_CODE  */
}

extern LoggingWithStorage sharedLogger;


void printRuntimeStats(void) {
	Logging *logger = &sharedLogger;

	scheduleMsg(logger, "maxSchedulingPrecisionLoss=%d", maxSchedulingPrecisionLoss);

#if EFI_CLOCK_LOCKS
	scheduleMsg(logger, "maxLockedDuration=%d / maxTriggerReentraint=%d", maxLockedDuration, maxTriggerReentraint);

	scheduleMsg(logger, "perSecondIrqDuration=%d ticks / perSecondIrqCounter=%d", perSecondIrqDuration, perSecondIrqCounter);
	scheduleMsg(logger, "IRQ CPU utilization %f%%", perSecondIrqDuration / (float)CORE_CLOCK * 100);

#endif /* EFI_CLOCK_LOCKS */

	scheduleMsg(logger, "maxEventCallbackDuration=%d", maxEventCallbackDuration);

#if EFI_HIP_9011
	scheduleMsg(logger, "hipLastExecutionCount=%d", hipLastExecutionCount);
#endif /* EFI_HIP_9011 */


}
