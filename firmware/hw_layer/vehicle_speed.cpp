/**
 * @file vehicle_speed.cpp
 *
 * @date Dec 26, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "vehicle_speed.h"

#if EFI_VEHICLE_SPEED

#include "engine.h"
#include "digital_input_icu.h"
#include "digital_input_exti.h"
#include "pin_repository.h"
#include "can_vss.h"

EXTERN_ENGINE;

static Logging *logger;

static efitick_t lastSignalTimeNt = 0;
static efitick_t vssDiff = 0;

#define DEFAULT_MOCK_SPEED -1
static float mockVehicleSpeed = DEFAULT_MOCK_SPEED; // in kilometers per hour

void setMockVehicleSpeed(float speedKPH) {
	mockVehicleSpeed = speedKPH;
}

/**
 * @return vehicle speed, in kilometers per hour
 */
float getVehicleSpeed(void) {
	if (mockVehicleSpeed != DEFAULT_MOCK_SPEED)
		return mockVehicleSpeed;
#if EFI_CAN_SUPPORT
	if (CONFIG(enableCanVss)) {
		return getVehicleCanSpeed();
	}
#endif	/* EFI_CAN_SUPPORT */	
	if (!hasVehicleSpeedSensor())
		return 0;
	efitick_t nowNt = getTimeNowNt();
	if (nowNt - lastSignalTimeNt > NT_PER_SECOND)
		return 0; // previous signal time is too long ago - we are stopped

	return engineConfiguration->vehicleSpeedCoef * NT_PER_SECOND / vssDiff;
}

static void vsAnaWidthCallback(void) {
	engine->engineState.vssEventCounter++;
	efitick_t nowNt = getTimeNowNt();
	vssDiff = nowNt - lastSignalTimeNt;
	lastSignalTimeNt = nowNt;
}

static void speedInfo(void) {
	scheduleMsg(logger, "VSS input at %s",
			hwPortname(CONFIG(vehicleSpeedSensorInputPin)));

	scheduleMsg(logger, "c=%.2f eventCounter=%d speed=%.2f",
			engineConfiguration->vehicleSpeedCoef,
			engine->engineState.vssEventCounter,
			getVehicleSpeed());
	scheduleMsg(logger, "vss diff %d", vssDiff);
}

bool hasVehicleSpeedSensor() {
	return CONFIG(vehicleSpeedSensorInputPin) != GPIO_UNASSIGNED;
}

#if HAL_VSS_USE_PAL
static void vsExtiCallback(void *) {
	vsAnaWidthCallback();
}
#endif /* HAL_VSS_USE_PAL */

void stopVSSPins(void) {
#if HAL_VSS_USE_PAL
	efiExtiDisablePin(activeConfiguration.vehicleSpeedSensorInputPin);
#elif HAL_USE_ICU
	stopDigitalCapture("VSS", activeConfiguration.vehicleSpeedSensorInputPin);
#endif /* HAL_VSS_USE_PAL, HAL_USE_ICU */
}

void startVSSPins(void) {
	if (!hasVehicleSpeedSensor())
		return;

#if HAL_VSS_USE_PAL
	ioline_t pal_line = PAL_LINE(getHwPort("vss", CONFIG(vehicleSpeedSensorInputPin)), getHwPin("vss", CONFIG(vehicleSpeedSensorInputPin)));
	efiExtiEnablePin("VSS", CONFIG(vehicleSpeedSensorInputPin), PAL_EVENT_MODE_BOTH_EDGES, vsExtiCallback, (void *)pal_line);
#elif HAL_USE_ICU
	digital_input_s* vehicleSpeedInput = startDigitalCapture("VSS", CONFIG(vehicleSpeedSensorInputPin));

	vehicleSpeedInput->widthListeners.registerCallback((VoidInt) vsAnaWidthCallback, NULL);
#else
	#error "HAL_USE_ICU or HAL_VSS_USE_PAL should be enabled to use EFI_VEHICLE_SPEED"
#endif /* HAL_VSS_USE_PAL, HAL_USE_ICU */
}

void initVehicleSpeed(Logging *l) {
	logger = l;
	addConsoleAction("speedinfo", speedInfo);
	startVSSPins();
}
#else  /* EFI_VEHICLE_SPEED */

#if EFI_UNIT_TEST
static float mockVehicleSpeed = 0.0; // in kilometers per hour

void setMockVehicleSpeed(float speedKPH) {
	mockVehicleSpeed = speedKPH;
}
float getVehicleSpeed(void) {
	
	// Mock return to be used in unit tests
	return mockVehicleSpeed;	
}
#else
float getVehicleSpeed(void) {
	
	// no VSS support
	return 0;	
}
#endif /* EFI_UNIT_TEST */
#endif /* EFI_VEHICLE_SPEED */
