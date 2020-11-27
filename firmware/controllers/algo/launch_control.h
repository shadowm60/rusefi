/*
 * @file launch_control.h
 *
 * @date Mar 23, 2020
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "engine_ptr.h"

class Logging;
void initLaunchControl(Logging *sharedLogger DECLARE_ENGINE_PARAMETER_SUFFIX);
void setDefaultLaunchParameters(DECLARE_CONFIG_PARAMETER_SIGNATURE);
void applyLaunchControlLimiting(bool *limitedSpark, bool *limitedFuel DECLARE_ENGINE_PARAMETER_SUFFIX);

class LaunchControlBase {
public:
	DECLARE_ENGINE_PTR;

	// Update the state of the launch control system
	void update();

	bool isInsideSpeedCondition() const;
	bool isInsideTpsCondition() const;
	bool isInsideSwitchCondition() const;
	bool isInsideRPMCondition(int rpm) const;

	bool isLaunchConditionMet(int rpm) const;

private:
	efitick_t launchTimer = 0;
};
