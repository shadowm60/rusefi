/**
 * @file	status_loop.h
 *
 * @date Mar 15, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "engine.h"

void updateDevConsoleState(void);
void prepareTunerStudioOutputs(void);
void startStatusThreads(void);
void initStatusLoop(void);

struct Writer;
/* function used to write current data to log file */
void writeLogLine(Writer& buffer);
/* function used to write current tooth log to file */
void writeToothLog(Writer& buffer);
void printOverallStatus(systime_t nowSeconds);
