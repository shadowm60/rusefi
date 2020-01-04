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
//#include "engine.h"
#include "tachometer.h"
#include "trigger_central.h"


EXTERN_ENGINE;


static scheduling_s tachTurnSignalOff;
static scheduling_s tachToggleSignalOff;
static scheduling_s tachToggleSignalOn;
static angle_t nextEventAngle;
/* used to detect configuration changes */
static uint8_t pulseNumbers;

static void toggleTachPinHigh(DECLARE_ENGINE_PARAMETER_SIGNATURE);
static void toggleTachPinLow(DECLARE_ENGINE_PARAMETER_SIGNATURE);

static void turnTachPinLow(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	enginePins.tachOut.setLow();
}

static void toggleTachPinLow(DECLARE_ENGINE_PARAMETER_SIGNATURE) {

	enginePins.tachOut.setLow();
	if (1 < engineConfiguration->tachPulsePerRev){
		scheduleByAngle(&tachToggleSignalOn, nextEventAngle,
					(schfunc_t) &toggleTachPinHigh, engine PASS_ENGINE_PARAMETER_SUFFIX);
	}


}

static void toggleTachPinHigh(DECLARE_ENGINE_PARAMETER_SIGNATURE) {

	enginePins.tachOut.setHigh();
	if (1 < engineConfiguration->tachPulsePerRev){
		scheduleByAngle(&tachToggleSignalOff, nextEventAngle,
							(schfunc_t) &toggleTachPinLow, engine PASS_ENGINE_PARAMETER_SUFFIX);

	}
}


static void tachSignalCallback(trigger_event_e ckpSignalType,
		uint32_t index DECLARE_ENGINE_PARAMETER_SUFFIX) {

#if EFI_UNIT_TEST
	printf("tachSignalCallback(%d %d)\n",ckpSignalType,index);
#else
	UNUSED(ckpSignalType);
#endif

	if (1 < engineConfiguration->tachPulsePerRev) {
		if (pulseNumbers != engineConfiguration->tachPulsePerRev){
			if (0 == index) {

				enginePins.tachOut.setHigh();
				pulseNumbers = engineConfiguration->tachPulsePerRev;

				nextEventAngle = 360/(engineConfiguration->tachPulsePerRev*2);
				scheduleByAngle(&tachToggleSignalOff, nextEventAngle,
    						(schfunc_t) &toggleTachPinLow, engine PASS_ENGINE_PARAMETER_SUFFIX);

			}
			else
			{
				return;
			}

		}
		else
		{
			return;
		}

	}
	else
	{
		if (index != (uint32_t)engineConfiguration->tachPulseTriggerIndex) {
			return;
		}
		pulseNumbers = 1;
		enginePins.tachOut.setHigh();
		float durationMs;
		if (engineConfiguration->tachPulseDurationAsDutyCycle) {
			// todo: implement tachPulseDurationAsDutyCycle
			durationMs = engineConfiguration->tachPulseDuractionMs;
		} else {
			durationMs = engineConfiguration->tachPulseDuractionMs;
		}
		engine->executor.scheduleForLater(&tachTurnSignalOff, (int)MS2US(durationMs), (schfunc_t) &turnTachPinLow, engine);

	}
}

void initTachometer(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	if (CONFIG(tachOutputPin) == GPIO_UNASSIGNED) {
		return;
	}
	pulseNumbers = 1;
	enginePins.tachOut.initPin("analog tach output", CONFIG(tachOutputPin), &CONFIG(tachOutputPinMode));

#if EFI_SHAFT_POSITION_INPUT
	addTriggerEventListener(tachSignalCallback, "tach", engine);
#endif /* EFI_SHAFT_POSITION_INPUT */
}



