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

static scheduling_s events[12];

struct tach_ctx {
	OutputPin *Pin;
	bool State;
};

static tach_ctx ctx_high;
static tach_ctx ctx_low;

static void setTach(void *param) {
	auto ctx = reinterpret_cast<tach_ctx *>(param);
	ctx->Pin->setValue(ctx->State);
}

static void tachSignalCallback(trigger_event_e ckpSignalType,
		uint32_t index DECLARE_ENGINE_PARAMETER_SUFFIX) {

	// TODO: does index 0 mean the beginning of a revolution, or of a cycle?

	// only process at index 0 - we schedule the full revolution all at once
	if (index != (uint32_t)engineConfiguration->tachPulseTriggerIndex) {
		return;
	}

#if EFI_UNIT_TEST
	printf("tachSignalCallback(%d %d)\n", ckpSignalType, index);
#else
	UNUSED(ckpSignalType);
#endif

	// TODO: warning if periods is set too high

	// How many tach pulse periods do we have?
	const int periods = engineConfiguration->tachPulsePerRev;
	// What is the angle per tach output period?
	const angle_t period = 360.0 / periods;

	float duty;

	if (engineConfiguration->tachPulseDurationAsDutyCycle) {
		// Simple case - duty explicitly set
		duty = engineConfiguration->tachPulseDuractionMs;
	} else {
		// Constant high-time mode - compute the correct duty cycle
		float cycleTimeMs = 60000.0 / GET_RPM();
		float periodTimeMs = cycleTimeMs / periods;

		duty = engineConfiguration->tachPulseDuractionMs / periodTimeMs;
	}

	// limit to 10..90% duty
	duty = maxF(0.1f, minF(duty, 0.9f));

	// Use duty to compute the angle widths of high/low periods
	angle_t angleHigh = period * duty;
	angle_t angleLow = period - angleHigh;

	angle_t angle = 0;

	for (int i = 0; i < periods; i++) {
		// Rising edge
		scheduleByAngle(&events[2 * i], angle, (schfunc_t)&setTach, &ctx_high PASS_ENGINE_PARAMETER_SUFFIX);
		angle += angleHigh;

		// Followed by falling edge
		scheduleByAngle(&events[2 * i + 1], angle, (schfunc_t)&setTach, &ctx_low PASS_ENGINE_PARAMETER_SUFFIX);
		angle += angleLow;
	}
}

void initTachometer(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	if (CONFIG(tachOutputPin) == GPIO_UNASSIGNED) {
		return;
	}

	enginePins.tachOut.initPin("analog tach output", CONFIG(tachOutputPin), &CONFIG(tachOutputPinMode));

	ctx_high.Pin = &enginePins.tachOut;
	ctx_high.State = true;

	ctx_low.Pin = &enginePins.tachOut;
	ctx_low.State = false;

#if EFI_SHAFT_POSITION_INPUT
	addTriggerEventListener(tachSignalCallback, "tach", engine);
#endif /* EFI_SHAFT_POSITION_INPUT */
}
