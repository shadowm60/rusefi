/**
 * @file    idle_hardware.h
 * @brief   Idle Air Control valve hardware
 *
 * @date November 3, 2020
 * 
 * This is just the hardware interface - deciding where to put the valve happens in idle_thread.cpp
 */

#pragma once

#include "engine.h"

void initIdleHardware(DECLARE_ENGINE_PARAMETER_SIGNATURE);
bool isIdleHardwareRestartNeeded();
bool isIdleMotorBusy(DECLARE_ENGINE_PARAMETER_SIGNATURE);
