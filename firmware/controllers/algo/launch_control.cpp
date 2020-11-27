/*
 * @file launch_control.cpp
 *
 *  @date 10. sep. 2019
 *      Author: Ola Ruud
 */

#include "engine.h"

#if EFI_LAUNCH_CONTROL
#include "boost_control.h"
#include "vehicle_speed.h"
#include "launch_control.h"
#include "io_pins.h"
#include "engine_configuration.h"
#include "engine_controller.h"
#include "periodic_task.h"
#include "pin_repository.h"
#include "allsensors.h"
#include "sensor.h"
#include "engine_math.h"
#include "efi_gpio.h"
#include "advance_map.h"
#include "engine_state.h"
#include "advance_map.h"

static Logging *logger;
static bool isInit = false;
static efitick_t launchTimer = 0;
static int retardThresholdRpm;

#if EFI_TUNER_STUDIO
#include "tunerstudio_outputs.h"
extern TunerStudioOutputChannels tsOutputChannels;
#endif /* EFI_TUNER_STUDIO */

EXTERN_ENGINE;


/**
 * We can have active condition from switch or from clutch.
 * In case we are dependent on VSS we just return true.
 */
bool isInsideSwitchCondition() {
	switch (CONFIG(launchActivationMode)) {
	case SWITCH_INPUT_LAUNCH:
		if (CONFIG(launchActivatePin) != GPIO_UNASSIGNED) {
			//todo: we should take into consideration if this sw is pulled high or low!
			engine->launchActivatePinState = efiReadPin(CONFIG(launchActivatePin));
		}
		return engine->launchActivatePinState;

	case CLUTCH_INPUT_LAUNCH:
		if (CONFIG(clutchDownPin) != GPIO_UNASSIGNED) {
			engine->clutchDownState = efiReadPin(CONFIG(clutchDownPin));
			
			if (CONFIG(clutchDownPinMode) == PI_PULLDOWN)
			{
				return !engine->clutchDownState;
			} else {
				return engine->clutchDownState;
			}
		} else {
			return false;
		}
		
	default:
		// ALWAYS_ACTIVE_LAUNCH
		return true;
	}
}

/**
 * Returns True in case Vehicle speed is less then trashold. 
 * This condiiion would only return true based on speed if DisablebySpeed is true
 * The condition logic is written in that way, that if we do not use disable by speed
 * then we have to return true, and trust that we would disable by other condition!
 */ 
bool isInsideSpeedCondition() {
	int speed = getVehicleSpeed();
	return (CONFIG(launchSpeedTreshold) > speed) || !engineConfiguration->launchDisableBySpeed;
}

/**
 * Returns false if TPS is invalid or TPS > preset trashold
 */
bool isInsideTpsCondition() {
	auto tps = Sensor::get(SensorType::DriverThrottleIntent);

	// Disallow launch without valid TPS
	if (!tps.Valid) {
		return false;
	}

	return CONFIG(launchTpsTreshold) < tps.Value;
}

/**
 * Condition is true as soon as we are above LaunchRpm
 */
bool isInsideRPMCondition(int rpm) {
	int launchRpm = CONFIG(launchRpm);
	return (launchRpm < rpm);
}

bool isLaunchConditionMet(int rpm) {

	bool activateSwitchCondition = isInsideSwitchCondition();
	bool rpmCondition = isInsideRPMCondition(rpm);
	bool speedCondition = isInsideSpeedCondition();
	bool tpsCondition = isInsideTpsCondition();

#if EFI_TUNER_STUDIO
	if (engineConfiguration->debugMode == DBG_LAUNCH) {
		tsOutputChannels.debugIntField1 = rpmCondition;
		tsOutputChannels.debugIntField2 = tpsCondition;
		tsOutputChannels.debugIntField3 = speedCondition;
		tsOutputChannels.debugIntField4 = activateSwitchCondition;
	}
#endif /* EFI_TUNER_STUDIO */

	return speedCondition && activateSwitchCondition && rpmCondition && tpsCondition;
}

void updateLaunchConditions(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	if (!CONFIG(launchControlEnabled)) {
		return;
	}

	if (!isInit) {
		return;
	}

	int rpm = GET_RPM();
	bool combinedConditions = isLaunchConditionMet(rpm);

	float timeDelay = CONFIG(launchActivateDelay);
	int cutRpmRange = CONFIG(hardCutRpmRange); //unused
	int launchAdvanceRpmRange = CONFIG(launchTimingRpmRange); //unused

	//recalculate in periodic task, this way we save time in applyLaunchControlLimiting
	//and still recalculat in case user changed the values
	retardThresholdRpm = CONFIG(launchRpm)+(CONFIG(enableLaunchRetard) ? \
	CONFIG(launchAdvanceRpmRange) : 0)+CONFIG(hardCutRpmRange);

	if (!combinedConditions) {
		// conditions not met, reset timer
		launchTimer = getTimeNowNt();
		engine->isLaunchCondition = false;
		engine->setLaunchBoostDuty = false;
		engine->applyLaunchControlRetard = false;
		engine->applyLaunchExtraFuel = false;
	} else {
		// If conditions are met...
		if ((getTimeNowNt() - launchTimer > MS2NT(timeDelay * 1000)) && combinedConditions) {
			engine->isLaunchCondition = true;           // ...enable launch!
			engine->applyLaunchExtraFuel = true;
		}
		if (CONFIG(enableLaunchBoost)) {
			engine->setLaunchBoostDuty = true;           // ...enable boost!
		}
		if (CONFIG(enableLaunchRetard)) {
			engine->applyLaunchControlRetard = true;    // ...enable retard!
		}
	}

#if EFI_TUNER_STUDIO
	if (CONFIG(debugMode) == DBG_LAUNCH) {
		tsOutputChannels.debugIntField5 = engine->clutchDownState;
		tsOutputChannels.debugFloatField1 = engine->launchActivatePinState;
		tsOutputChannels.debugFloatField2 = engine->isLaunchCondition;
		tsOutputChannels.debugFloatField3 = combinedConditions;
	}
#endif /* EFI_TUNER_STUDIO */
}

void setDefaultLaunchParameters(DECLARE_CONFIG_PARAMETER_SIGNATURE) {
	engineConfiguration->launchRpm = 4000;    // Rpm to trigger Launch condition
	engineConfiguration->launchTimingRetard = 10; // retard in absolute degrees ATDC
	engineConfiguration->launchTimingRpmRange = 500; // Rpm above Launch triggered for full retard
	engineConfiguration->launchSparkCutEnable = true;
	engineConfiguration->launchFuelCutEnable = false;
	engineConfiguration->hardCutRpmRange = 500; //Rpm above Launch triggered +(if retard enabled) launchTimingRpmRange to hard cut
	engineConfiguration->launchSpeedTreshold = 10; //maximum speed allowed before disable launch
	engineConfiguration->launchFuelAdded = 10; // Extra fuel in % when launch are triggered
	engineConfiguration->launchBoostDuty = 70; // boost valve duty cycle at launch
	engineConfiguration->launchActivateDelay = 3; // Delay in seconds for launch to kick in
	engineConfiguration->enableLaunchRetard = true;
	engineConfiguration->enableLaunchBoost = true;
	engineConfiguration->launchSmoothRetard = true; //interpolates the advance linear from launchrpm to fully retarded at launchtimingrpmrange
	engineConfiguration->antiLagRpmTreshold = 3000;
}

void applyLaunchControlLimiting(bool *limitedSpark, bool *limitedFuel DECLARE_ENGINE_PARAMETER_SUFFIX) {

	if (retardThresholdRpm < GET_RPM()) {
		*limitedSpark = engineConfiguration->launchSparkCutEnable;
		*limitedFuel = engineConfiguration->launchFuelCutEnable;
	}
}

void initLaunchControl(Logging *sharedLogger DECLARE_ENGINE_PARAMETER_SUFFIX) {
	logger = sharedLogger;
	retardThresholdRpm = CONFIG(launchRpm)+(CONFIG(enableLaunchRetard) ? \
	CONFIG(launchAdvanceRpmRange) : 0) +CONFIG(hardCutRpmRange);
	isInit = true;
}

#endif /* EFI_LAUNCH_CONTROL */
