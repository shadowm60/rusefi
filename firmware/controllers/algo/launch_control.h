/*
 * @file launch_control.h
 *
 * @date Mar 23, 2020
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "engine.h"
#include "periodic_task.h"

class LaunchControl : public PeriodicTimerController {
public:
	DECLARE_ENGINE_PTR;

	int getPeriodMs() override;
	void PeriodicTask() override;

private:
    efitick_t launchTimer;
};

void initLaunchControl(Logging *sharedLogger DECLARE_ENGINE_PARAMETER_SUFFIX);
void setDefaultLaunchParameters(DECLARE_CONFIG_PARAMETER_SIGNATURE);
void applyLaunchControlLimiting(bool *limitedSpark, bool *limitedFuel DECLARE_ENGINE_PARAMETER_SUFFIX);
