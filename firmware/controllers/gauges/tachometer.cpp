/*
 * @file tachometer.cpp
 * @brief This is about driving external analog tachometers
 *
 * This implementation produces one pulse per engine cycle
 *
 * todo: these is a bit of duplication with dizzySparkOutputPin
 *
 * @date Aug 18, 2015
 * @author Andrey Belomutskiy, (c) 2012-2018
 */

#include "tachometer.h"
#include "trigger_central.h"

#if !EFI_UNIT_TEST

EXTERN_ENGINE;

static scheduling_s tachTurnSignalOff;
static float halfperiodMs = 4; /* period in ms, used for multi pulse output, actually half period */
static uint8_t isStarted = 0;

static void toggleTachPinHigh(void);
static void toggleTachPinLow(void);

static void turnTachPinLow(void) {
	enginePins.tachOut.setLow();
}

static void toggleTachPinLow(void)
{
	if(isStarted)
	{
		enginePins.tachOut.setLow();
		engine->executor.scheduleForLater(&tachTurnSignalOff, (int)MS2US(halfperiodMs), (schfunc_t) &toggleTachPinHigh, NULL);
	}
}

static void toggleTachPinHigh(void)
{
	if(isStarted)
	{
		enginePins.tachOut.setHigh();
		engine->executor.scheduleForLater(&tachTurnSignalOff, (int)MS2US(halfperiodMs), (schfunc_t) &toggleTachPinLow, NULL);
	}

}

static void tachSignalCallback(trigger_event_e ckpSignalType,
		uint32_t index DECLARE_ENGINE_PARAMETER_SUFFIX) {
	UNUSED(ckpSignalType);

	if (index != (uint32_t)engineConfiguration->tachPulseTriggerIndex) {
		return;
	}

	if ((1 < engineConfiguration->tachPulsePerRev))
	{

		/* reset timing counters */
		int rpm = GET_RPM();
		float duration = 60000/rpm;
		halfperiodMs = (duration/(engineConfiguration->tachPulsePerRev*2));
        if(isStarted==0)
        {
        	enginePins.tachOut.setHigh();
        	isStarted = 1;
        	engine->executor.scheduleForLater(&tachTurnSignalOff, (int)MS2US(halfperiodMs), (schfunc_t) &toggleTachPinLow, NULL);
        }
	}
	else
	{
		isStarted = 0;

		enginePins.tachOut.setHigh();
		float durationMs;
		if (engineConfiguration->tachPulseDurationAsDutyCycle) {
			// todo: implement tachPulseDurationAsDutyCycle
			durationMs = engineConfiguration->tachPulseDuractionMs;
		} else {
			durationMs = engineConfiguration->tachPulseDuractionMs;
		}
		engine->executor.scheduleForLater(&tachTurnSignalOff, (int)MS2US(durationMs), (schfunc_t) &turnTachPinLow, NULL);
	}
}

void initTachometer(void) {
	if (CONFIG(tachOutputPin) == GPIO_UNASSIGNED) {
		return;
	}

	enginePins.tachOut.initPin("analog tach output", CONFIG(tachOutputPin), &CONFIG(tachOutputPinMode));

#if EFI_SHAFT_POSITION_INPUT
	addTriggerEventListener(tachSignalCallback, "tach", engine);
#endif /* EFI_SHAFT_POSITION_INPUT */
}

#endif /* EFI_UNIT_TEST */
