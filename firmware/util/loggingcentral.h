/**
 * @file	loggingcentral.h
 *
 * @date Mar 8, 2015
 * @author Andrey Belomutskiy, (c) 2012-2020
 */
#pragma once

class Logging;

const char* swapOutputBuffers(size_t* actualOutputBufferSize);
void scheduleMsg(Logging *logging, const char *fmt, ...);
