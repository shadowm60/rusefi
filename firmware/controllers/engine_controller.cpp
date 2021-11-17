/**
 * @file    engine_controller.cpp
 * @brief   Controllers package entry point code
 *
 *
 *
 * @date Feb 7, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "global.h"
#include "os_access.h"
#include "trigger_central.h"
#include "engine_controller.h"
#include "fsio_core.h"
#include "fsio_impl.h"
#include "idle_thread.h"
#include "advance_map.h"
#include "rpm_calculator.h"
#include "main_trigger_callback.h"
#include "io_pins.h"
#include "flash_main.h"
#include "bench_test.h"
#include "os_util.h"
#include "engine_math.h"
#include "allsensors.h"
#include "electronic_throttle.h"
#include "map_averaging.h"
#include "malfunction_central.h"
#include "malfunction_indicator.h"
#include "speed_density.h"
#include "local_version_holder.h"
#include "alternator_controller.h"
#include "fuel_math.h"
#include "settings.h"
#include "aux_pid.h"
#include "spark_logic.h"
#include "aux_valves.h"
#include "accelerometer.h"
#include "counter64.h"
#include "perf_trace.h"
#include "boost_control.h"
#include "launch_control.h"
#include "tachometer.h"

#if EFI_SENSOR_CHART
#include "sensor_chart.h"
#endif /* EFI_SENSOR_CHART */

#if EFI_TUNER_STUDIO
#include "tunerstudio.h"
#endif /* EFI_TUNER_STUDIO */

#if EFI_LOGIC_ANALYZER
#include "logic_analyzer.h"
#endif /* EFI_LOGIC_ANALYZER */

#if HAL_USE_ADC
#include "AdcConfiguration.h"
#endif /* HAL_USE_ADC */

#if defined(EFI_BOOTLOADER_INCLUDE_CODE)
#include "bootloader/bootloader.h"
#endif /* EFI_BOOTLOADER_INCLUDE_CODE */

#if EFI_PROD_CODE || EFI_SIMULATOR
#include "periodic_task.h"
#endif

#if ! EFI_UNIT_TEST
#include "init.h"
#endif /* EFI_UNIT_TEST */

#if EFI_PROD_CODE
#include "pwm_generator.h"
#include "adc_inputs.h"

#include "pwm_tester.h"
#include "pwm_generator.h"
#include "lcd_controller.h"
#include "pin_repository.h"
#endif /* EFI_PROD_CODE */

#if EFI_CJ125
#include "cj125.h"
#endif /* EFI_CJ125 */

EXTERN_ENGINE;

#if !EFI_UNIT_TEST

static LoggingWithStorage logger("Engine Controller");

/**
 * todo: this should probably become 'static', i.e. private, and propagated around explicitly?
 */
Engine ___engine CCM_OPTIONAL;

#else // EFI_UNIT_TEST

Engine * engine;

#endif /* EFI_UNIT_TEST */


void initDataStructures() {
#if EFI_ENGINE_CONTROL
	initFuelMap();
	initTimingMap();
	initSpeedDensity();
#endif // EFI_ENGINE_CONTROL
}

static void mostCommonInitEngineController(Logging *sharedLogger DECLARE_ENGINE_PARAMETER_SUFFIX) {
#if !EFI_UNIT_TEST
	initNewSensors(sharedLogger);
#endif /* EFI_UNIT_TEST */

	initSensors(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);

	initAccelEnrichment(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);

#if EFI_FSIO
	initFsioImpl(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);
#endif /* EFI_FSIO */

#if EFI_IDLE_CONTROL
	startIdleThread(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);
#endif /* EFI_IDLE_CONTROL */

#if EFI_ELECTRONIC_THROTTLE_BODY
	initElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE);
#endif /* EFI_ELECTRONIC_THROTTLE_BODY */

#if EFI_MAP_AVERAGING
	if (engineConfiguration->isMapAveragingEnabled) {
		initMapAveraging(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);
	}
#endif /* EFI_MAP_AVERAGING */

#if EFI_BOOST_CONTROL
	initBoostCtrl(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);
#endif /* EFI_BOOST_CONTROL */

#if EFI_LAUNCH_CONTROL
	initLaunchControl(sharedLogger PASS_ENGINE_PARAMETER_SUFFIX);
#endif

}

#if EFI_ENABLE_MOCK_ADC

static void initMockVoltage(void) {
#if EFI_SIMULATOR
	setMockCltVoltage(2);
	setMockIatVoltage(2);
#endif /* EFI_SIMULATOR */
}

#endif /* EFI_ENABLE_MOCK_ADC */


#if !EFI_UNIT_TEST

static void doPeriodicSlowCallback();

class PeriodicFastController : public PeriodicTimerController {
	void PeriodicTask() override {
		engine->periodicFastCallback();
	}

	int getPeriodMs() override {
		return FAST_CALLBACK_PERIOD_MS;
	}
};

class PeriodicSlowController : public PeriodicTimerController {
	void PeriodicTask() override {
		doPeriodicSlowCallback();
	}

	int getPeriodMs() override {
		// we need at least protection from zero value while resetting configuration
		int periodMs = maxI(50, CONFIG(generalPeriodicThreadPeriodMs));
		return periodMs;
	}
};

static PeriodicFastController fastController;
static PeriodicSlowController slowController;

class EngineStateBlinkingTask : public PeriodicTimerController {
	int getPeriodMs() override {
		return 50;
	}

	void PeriodicTask() override {
		counter++;
#if EFI_SHAFT_POSITION_INPUT
		bool is_running = engine->rpmCalculator.isRunning();
#else
		bool is_running = false;
#endif /* EFI_SHAFT_POSITION_INPUT */

		if (is_running) {
			// blink in running mode
			enginePins.runningLedPin.setValue(counter % 2);
		} else {
			int is_cranking = engine->rpmCalculator.isCranking();
			enginePins.runningLedPin.setValue(is_cranking);
		}
	}
private:
	int counter = 0;
};

static EngineStateBlinkingTask engineStateBlinkingTask;

/**
 * 32 bit return type overflows in 23 days. I think we do not expect rusEFI to run for 23 days straight days any time soon?
 */
efitimems_t currentTimeMillis(void) {
	return US2MS(getTimeNowUs());
}

/**
 * Integer number of seconds since ECU boot.
 * 31,710 years - would not overflow during our life span.
 */
efitimesec_t getTimeNowSeconds(void) {
	return getTimeNowUs() / US_PER_SECOND;
}

static void resetAccel() {
	engine->tpsAccelEnrichment.resetAE();

	for (unsigned int i = 0; i < sizeof(engine->wallFuel) / sizeof(engine->wallFuel[0]); i++)
	{
		engine->wallFuel[i].resetWF();
	}
}

static void doPeriodicSlowCallback() {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	efiAssertVoid(CUSTOM_ERR_6661, getCurrentRemainingStack() > 64, "lowStckOnEv");

	slowStartStopButtonCallback();

		bool wasStarterEngaged = enginePins.starterControl.getAndSet(1);
		if (!wasStarterEngaged) {
			scheduleMsg(&logger, "Let's crank this engine for up to %dseconds!", CONFIG(startCrankingDuration));
		}
	} else if (engine->rpmCalculator.isRunning(PASS_ENGINE_PARAMETER_SIGNATURE)) {
		scheduleMsg(&logger, "Let's stop this engine!");
		scheduleStopEngine();
	}
}

static void slowStartStopButtonCallback(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	if (CONFIG(startStopButtonPin) != GPIO_UNASSIGNED) {
#if EFI_PROD_CODE
		bool startStopState = efiReadPin(CONFIG(startStopButtonPin));

		if (startStopState && !engine->startStopState) {
			// we are here on transition from 0 to 1
			onStartStopButtonToggle(PASS_ENGINE_PARAMETER_SIGNATURE);
		}
		engine->startStopState = startStopState;
#endif /* EFI_PROD_CODE */
	}

	if (engine->startStopStateLastPushTime == 0) {
		// nothing is going on with startStop button
		return;
	}


	// todo: should this be simply FSIO?
	if (engine->rpmCalculator.isRunning(PASS_ENGINE_PARAMETER_SIGNATURE)) {
		// turn starter off once engine is running
		bool wasStarterEngaged = enginePins.starterControl.getAndSet(0);
		if (wasStarterEngaged) {
			scheduleMsg(&logger, "Engine runs we can disengage the starter");
		}
		engine->startStopStateLastPushTime = 0;
	}

	if (getTimeNowNt() - engine->startStopStateLastPushTime > NT_PER_SECOND * CONFIG(startCrankingDuration)) {
		bool wasStarterEngaged = enginePins.starterControl.getAndSet(0);
		if (wasStarterEngaged) {
			scheduleMsg(&logger, "Cranking timeout %dseconds", CONFIG(startCrankingDuration));
		}
		engine->startStopStateLastPushTime = 0;
	}
}

static void doPeriodicSlowCallback(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	efiAssertVoid(CUSTOM_ERR_6661, getCurrentRemainingStack() > 64, "lowStckOnEv");
#if EFI_PROD_CODE
	touchTimeCounter();
#endif /* EFI_PROD_CODE */

	slowStartStopButtonCallback(PASS_ENGINE_PARAMETER_SIGNATURE);

	/**
	 * Update engine RPM state if needed (check timeouts).
	 */
	bool isSpinning = engine->rpmCalculator.checkIfSpinning(nowNt);
	if (!isSpinning) {
		engine->rpmCalculator.setStopSpinning();
	}

	if (engine->directSelfStimulation || engine->rpmCalculator.isStopped()) {
		/**
		 * rusEfi usually runs on hardware which halts execution while writing to internal flash, so we
		 * postpone writes to until engine is stopped. Writes in case of self-stimulation are fine.
		 *
		 * todo: allow writing if 2nd bank of flash is used
		 */
#if EFI_INTERNAL_FLASH
		writeToFlashIfPending();
#endif /* EFI_INTERNAL_FLASH */
		resetAccel();
	} else {
		updatePrimeInjectionPulseState();
	}

	if (engine->versionForConfigurationListeners.isOld(engine->getGlobalConfigurationVersion())) {
		updateAccelParameters();
	}

	engine->periodicSlowCallback();
#endif /* if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT */

	if (engineConfiguration->tcuEnabled) {
		engine->gearController->update();
	}
}

void initPeriodicEvents() {
	slowController.Start();
	fastController.Start();
}

char * getPinNameByAdcChannel(const char *msg, adc_channel_e hwChannel, char *buffer) {
#if HAL_USE_ADC
	if (hwChannel == EFI_ADC_NONE) {
		strcpy(buffer, "NONE");
	} else {
		strcpy((char*) buffer, portname(getAdcChannelPort(msg, hwChannel)));
		itoa10(&buffer[2], getAdcChannelPin(hwChannel));
	}
#else
	strcpy(buffer, "NONE");
#endif /* HAL_USE_ADC */
	return (char*) buffer;
}

static char pinNameBuffer[16];

#if HAL_USE_ADC
extern AdcDevice fastAdc;
#endif /* HAL_USE_ADC */

static void printAnalogChannelInfoExt(const char *name, adc_channel_e hwChannel, float adcVoltage,
		float dividerCoeff) {
#if HAL_USE_ADC
	if (hwChannel == EFI_ADC_NONE) {
		scheduleMsg(&logger, "ADC is not assigned for %s", name);
		return;
	}

	if (fastAdc.isHwUsed(hwChannel)) {
		scheduleMsg(&logger, "fast enabled=%s", boolToString(CONFIG(isFastAdcEnabled)));
	}

	float voltage = adcVoltage * dividerCoeff;
	scheduleMsg(&logger, "%s ADC%d %s %s adc=%.2f/input=%.2fv/divider=%.2f", name, hwChannel, getAdcMode(hwChannel),
			getPinNameByAdcChannel(name, hwChannel, pinNameBuffer), adcVoltage, voltage, dividerCoeff);
#endif /* HAL_USE_ADC */
}

static void printAnalogChannelInfo(const char *name, adc_channel_e hwChannel) {
#if HAL_USE_ADC
	printAnalogChannelInfoExt(name, hwChannel, getVoltage(name, hwChannel), engineConfiguration->analogInputDividerCoefficient);
#endif /* HAL_USE_ADC */
}

static void printAnalogInfo() {
	efiPrintf("analogInputDividerCoefficient: %.2f", engineConfiguration->analogInputDividerCoefficient);

	printAnalogChannelInfo("hip9011", engineConfiguration->hipOutputChannel);
	printAnalogChannelInfo("fuel gauge", engineConfiguration->fuelLevelSensor);
	printAnalogChannelInfo("TPS", engineConfiguration->tps1_1AdcChannel);
	printAnalogChannelInfo("TPS2", engineConfiguration->tps2_1AdcChannel);
	printAnalogChannelInfo("pPS", engineConfiguration->throttlePedalPositionAdcChannel);
	printAnalogChannelInfo("CLT", engineConfiguration->clt.adcChannel);
	printAnalogChannelInfo("IAT", engineConfiguration->iat.adcChannel);
	printAnalogChannelInfo("MAF", engineConfiguration->mafAdcChannel);
	for (int i = 0; i < FSIO_ANALOG_INPUT_COUNT ; i++) {
		adc_channel_e ch = engineConfiguration->fsioAdc[i];
		printAnalogChannelInfo("fsio", ch);
	}

	printAnalogChannelInfo("AFR", engineConfiguration->afr.hwChannel);
	printAnalogChannelInfo("MAP", engineConfiguration->map.sensor.hwChannel);
	printAnalogChannelInfo("BARO", engineConfiguration->baroSensor.hwChannel);
	printAnalogChannelInfo("extKno", engineConfiguration->externalKnockSenseAdc);

	printAnalogChannelInfo("OilP", engineConfiguration->oilPressure.hwChannel);

	printAnalogChannelInfo("CJ UR", engineConfiguration->cj125ur);
	printAnalogChannelInfo("CJ UA", engineConfiguration->cj125ua);

	printAnalogChannelInfo("A/C sw", engineConfiguration->acSwitchAdc);
	printAnalogChannelInfo("HIP9011", engineConfiguration->hipOutputChannel);

	printAnalogChannelInfoExt("Vbatt", engineConfiguration->vbattAdcChannel, getVoltage("vbatt", engineConfiguration->vbattAdcChannel),
			engineConfiguration->vbattDividerCoeff);
}

#define isOutOfBounds(offset) ((offset<0) || (offset) >= (int) sizeof(engine_configuration_s))

static void getShort(int offset) {
	if (isOutOfBounds(offset))
		return;
	uint16_t *ptr = (uint16_t *) (&((char *) engineConfiguration)[offset]);
	uint16_t value = *ptr;
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "short%s%d is %d", CONSOLE_DATA_PROTOCOL_TAG, offset, value);
}

static void getByte(int offset) {
	if (isOutOfBounds(offset))
		return;
	uint8_t *ptr = (uint8_t *) (&((char *) engineConfiguration)[offset]);
	uint8_t value = *ptr;
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "byte%s%d is %d", CONSOLE_DATA_PROTOCOL_TAG, offset, value);
}

static void onConfigurationChanged() {
#if EFI_TUNER_STUDIO
	// on start-up rusEfi would read from working copy of TS while
	// we have a lot of console commands which write into real copy of configuration directly
	// we have a bit of a mess here
	syncTunerStudioCopy();
#endif /* EFI_TUNER_STUDIO */
	incrementGlobalConfigurationVersion();
}

static void setBit(const char *offsetStr, const char *bitStr, const char *valueStr) {
	int offset = atoi(offsetStr);
	if (absI(offset) == absI(ERROR_CODE)) {
		scheduleMsg(&logger, "invalid offset [%s]", offsetStr);
		return;
	}
	if (isOutOfBounds(offset)) {
		return;
	}
	int bit = atoi(bitStr);
	if (absI(bit) == absI(ERROR_CODE)) {
		scheduleMsg(&logger, "invalid bit [%s]", bitStr);
		return;
	}
	int value = atoi(valueStr);
	if (absI(value) == absI(ERROR_CODE)) {
		scheduleMsg(&logger, "invalid value [%s]", valueStr);
		return;
	}
	int *ptr = (int *) (&((char *) engineConfiguration)[offset]);
	*ptr ^= (-value ^ *ptr) & (1 << bit);
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "bit%s%d/%d is %d", CONSOLE_DATA_PROTOCOL_TAG, offset, bit, value);
	onConfigurationChanged();
}

static void setShort(const int offset, const int value) {
	if (isOutOfBounds(offset))
		return;
	uint16_t *ptr = (uint16_t *) (&((char *) engineConfiguration)[offset]);
	*ptr = (uint16_t) value;
	getShort(offset);
	onConfigurationChanged();
}

static void setByte(const int offset, const int value) {
	if (isOutOfBounds(offset))
		return;
	uint8_t *ptr = (uint8_t *) (&((char *) engineConfiguration)[offset]);
	*ptr = (uint8_t) value;
	getByte(offset);
	onConfigurationChanged();
}

static void getBit(int offset, int bit) {
	if (isOutOfBounds(offset))
		return;
	int *ptr = (int *) (&((char *) engineConfiguration)[offset]);
	int value = (*ptr >> bit) & 1;
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "bit%s%d/%d is %d", CONSOLE_DATA_PROTOCOL_TAG, offset, bit, value);
}

static void getInt(int offset) {
	if (isOutOfBounds(offset))
		return;
	int *ptr = (int *) (&((char *) engineConfiguration)[offset]);
	int value = *ptr;
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "int%s%d is %d", CONSOLE_DATA_PROTOCOL_TAG, offset, value);
}

static void setInt(const int offset, const int value) {
	if (isOutOfBounds(offset))
		return;
	int *ptr = (int *) (&((char *) engineConfiguration)[offset]);
	*ptr = value;
	getInt(offset);
	onConfigurationChanged();
}

static void getFloat(int offset) {
	if (isOutOfBounds(offset))
		return;
	float *ptr = (float *) (&((char *) engineConfiguration)[offset]);
	float value = *ptr;
	/**
	 * this response is part of rusEfi console API
	 */
	scheduleMsg(&logger, "float%s%d is %.5f", CONSOLE_DATA_PROTOCOL_TAG, offset, value);
}

static void setFloat(const char *offsetStr, const char *valueStr) {
	int offset = atoi(offsetStr);
	if (absI(offset) == absI(ERROR_CODE)) {
		scheduleMsg(&logger, "invalid offset [%s]", offsetStr);
		return;
	}
	if (isOutOfBounds(offset))
		return;
	float value = atoff(valueStr);
	if (cisnan(value)) {
		scheduleMsg(&logger, "invalid value [%s]", valueStr);
		return;
	}
	float *ptr = (float *) (&((char *) engineConfiguration)[offset]);
	*ptr = value;
	getFloat(offset);
	onConfigurationChanged();
}

static void initConfigActions() {
	addConsoleActionSS("set_float", (VoidCharPtrCharPtr) setFloat);
	addConsoleActionII("set_int", (VoidIntInt) setInt);
	addConsoleActionII("set_short", (VoidIntInt) setShort);
	addConsoleActionII("set_byte", (VoidIntInt) setByte);
	addConsoleActionSSS("set_bit", setBit);

	addConsoleActionI("get_float", getFloat);
	addConsoleActionI("get_int", getInt);
	addConsoleActionI("get_short", getShort);
	addConsoleActionI("get_byte", getByte);
	addConsoleActionII("get_bit", getBit);
}

// todo: move this logic somewhere else?
static void getKnockInfo(void) {
	adc_channel_e hwChannel = engineConfiguration->externalKnockSenseAdc;
	scheduleMsg(&logger, "externalKnockSenseAdc on ADC", getPinNameByAdcChannel("knock", hwChannel, pinNameBuffer));

	engine->printKnockState();
}
#endif /* EFI_UNIT_TEST */

// this method is used by real firmware and simulator and unit test
void commonInitEngineController() {
	initInterpolation();

#if EFI_SIMULATOR
	printf("commonInitEngineController\n");
#endif

#if !EFI_UNIT_TEST
	initConfigActions();
#endif /* EFI_UNIT_TEST */

#if EFI_ENGINE_CONTROL
	/**
	 * This has to go after 'enginePins.startPins()' in order to
	 * properly detect un-assigned output pins
	 */
	prepareShapes();
#endif /* EFI_PROD_CODE && EFI_ENGINE_CONTROL */


#if EFI_ENABLE_MOCK_ADC
	initMockVoltage();
#endif /* EFI_ENABLE_MOCK_ADC */

#if EFI_SENSOR_CHART
	initSensorChart();
#endif /* EFI_SENSOR_CHART */


#if EFI_TUNER_STUDIO
	if (engineConfiguration->isTunerStudioEnabled) {
		startTunerStudioConnectivity();
	}
#endif /* EFI_TUNER_STUDIO */

#if EFI_PROD_CODE || EFI_SIMULATOR
	initSettings();

	if (hasFirmwareError()) {
		return;
	}
#endif

#if !EFI_UNIT_TEST
	// This is tested independently - don't configure sensors for tests.
	// This lets us selectively mock them for each test.
	initNewSensors();
#endif /* EFI_UNIT_TEST */

	initSensors();

	initAccelEnrichment();

#if EFI_FSIO
	initFsioImpl();
#endif /* EFI_FSIO */

	initGpPwm();

#if EFI_IDLE_CONTROL
	startIdleThread();
#endif /* EFI_IDLE_CONTROL */

	initButtonShift();

	initButtonDebounce();
	initStartStopButton();

#if EFI_ELECTRONIC_THROTTLE_BODY
	initElectronicThrottle();
#endif /* EFI_ELECTRONIC_THROTTLE_BODY */

#if EFI_MAP_AVERAGING
	if (engineConfiguration->isMapAveragingEnabled) {
		initMapAveraging();
	}
#endif /* EFI_MAP_AVERAGING */

#if EFI_BOOST_CONTROL
	initBoostCtrl();
#endif /* EFI_BOOST_CONTROL */

#if EFI_LAUNCH_CONTROL
	initLaunchControl();
#endif

#if EFI_SHAFT_POSITION_INPUT
	/**
	 * there is an implicit dependency on the fact that 'tachometer' listener is the 1st listener - this case
	 * other listeners can access current RPM value
	 */
	initRpmCalculator();
#endif /* EFI_SHAFT_POSITION_INPUT */

#if (EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT) || EFI_SIMULATOR || EFI_UNIT_TEST
	if (engineConfiguration->isEngineControlEnabled) {
		initAuxValves();
		/**
		 * This method adds trigger listener which actually schedules ignition
		 */
		initMainEventListener();
#if EFI_HPFP
		initHPFP();
#endif // EFI_HPFP
	}
#endif /* EFI_ENGINE_CONTROL */

	initTachometer();
}

// Returns false if there's an obvious problem with the loaded configuration
bool validateConfig() {
	if (engineConfiguration->specs.cylindersCount > MAX_CYLINDER_COUNT) {
		firmwareError(OBD_PCM_Processor_Fault, "Invalid cylinder count: %d", engineConfiguration->specs.cylindersCount);
		return false;
	}

	// Fueling
	{
		ensureArrayIsAscending("VE load", config->veLoadBins);
		ensureArrayIsAscending("VE RPM", config->veRpmBins);

		ensureArrayIsAscending("Lambda/AFR load", config->lambdaLoadBins);
		ensureArrayIsAscending("Lambda/AFR RPM", config->lambdaRpmBins);

		ensureArrayIsAscending("Fuel CLT mult", config->cltFuelCorrBins);
		ensureArrayIsAscending("Fuel IAT mult", config->iatFuelCorrBins);

		ensureArrayIsAscending("Injection phase load", config->injPhaseLoadBins);
		ensureArrayIsAscending("Injection phase RPM", config->injPhaseRpmBins);

		ensureArrayIsAscending("TPS/TPS AE from", config->tpsTpsAccelFromRpmBins);
		ensureArrayIsAscending("TPS/TPS AE to", config->tpsTpsAccelToRpmBins);
	}

	// Ignition
	{
		ensureArrayIsAscending("Dwell RPM", engineConfiguration->sparkDwellRpmBins);

		ensureArrayIsAscending("Ignition load", config->ignitionLoadBins);
		ensureArrayIsAscending("Ignition RPM", config->ignitionRpmBins);

		ensureArrayIsAscending("Ignition CLT corr", engineConfiguration->cltTimingBins);

		ensureArrayIsAscending("Ignition IAT corr IAT", config->ignitionIatCorrLoadBins);
		ensureArrayIsAscending("Ignition IAT corr RPM", config->ignitionIatCorrRpmBins);
	}

	if (config->mapEstimateTpsBins[1] != 0) { // only validate map if not all zeroes default
		ensureArrayIsAscending("Map estimate TPS", config->mapEstimateTpsBins);
	}

	if (config->mapEstimateRpmBins[1] != 0) { // only validate map if not all zeroes default
		ensureArrayIsAscending("Map estimate RPM", config->mapEstimateRpmBins);
	}

	ensureArrayIsAscending("MAF decoding", config->mafDecodingBins);

	// Cranking tables
	ensureArrayIsAscending("Cranking fuel mult", config->crankingFuelBins);
	ensureArrayIsAscending("Cranking duration", config->crankingCycleBins);
	ensureArrayIsAscending("Cranking TPS", engineConfiguration->crankingTpsBins);

	// Idle tables
	ensureArrayIsAscending("Idle target RPM", engineConfiguration->cltIdleRpmBins);
	ensureArrayIsAscending("Idle warmup mult", config->cltIdleCorrBins);
	if (engineConfiguration->iacCoastingBins[1] != 0) { // only validate map if not all zeroes default
		ensureArrayIsAscending("Idle coasting position", engineConfiguration->iacCoastingBins);
	}
	if (config->idleVeBins[1] != 0) { // only validate map if not all zeroes default
		ensureArrayIsAscending("Idle VE", config->idleVeBins);
	}
	if (config->idleAdvanceBins[1] != 0) { // only validate map if not all zeroes default
		ensureArrayIsAscending("Idle timing", config->idleAdvanceBins);
	}

	for (size_t index = 0; index < efi::size(engineConfiguration->vrThreshold); index++) {
		auto& cfg = engineConfiguration->vrThreshold[index];

		if (cfg.pin == GPIO_UNASSIGNED) {
			continue;
		}
		ensureArrayIsAscending("VR Bins", cfg.rpmBins);
		ensureArrayIsAscending("VR values", cfg.values);
	}

	// Boost
	ensureArrayIsAscending("Boost control TPS", config->boostTpsBins);
	ensureArrayIsAscending("Boost control RPM", config->boostRpmBins);

	// ETB
	ensureArrayIsAscending("Pedal map pedal", config->pedalToTpsPedalBins);
	ensureArrayIsAscending("Pedal map RPM", config->pedalToTpsRpmBins);

	// VVT
	if (engineConfiguration->camInputs[0] != GPIO_UNASSIGNED) {
		ensureArrayIsAscending("VVT intake load", config->vvtTable1LoadBins);
		ensureArrayIsAscending("VVT intake RPM", config->vvtTable1RpmBins);
	}

#if CAM_INPUTS_COUNT != 1
	if (engineConfiguration->camInputs[1] != GPIO_UNASSIGNED) {
		ensureArrayIsAscending("VVT exhaust load", config->vvtTable2LoadBins);
		ensureArrayIsAscending("VVT exhaust RPM", config->vvtTable2RpmBins);
	}
#endif

	return true;
}

#if !EFI_UNIT_TEST

void initEngineContoller() {
	addConsoleAction("analoginfo", printAnalogInfo);

#if EFI_PROD_CODE && EFI_ENGINE_CONTROL
	enginePins.startPins();

	initBenchTest(sharedLogger);
#endif /* EFI_PROD_CODE && EFI_ENGINE_CONTROL */

	commonInitEngineController(sharedLogger);

#if EFI_PROD_CODE
	initPwmGenerator();
#endif

#if EFI_LOGIC_ANALYZER
	if (engineConfiguration->isWaveAnalyzerEnabled) {
		initWaveAnalyzer(sharedLogger);
	}
#endif /* EFI_LOGIC_ANALYZER */

#if EFI_CJ125
	/**
	 * this uses SimplePwm which depends on scheduler, has to be initialized after scheduler
	 */
	initCJ125();
#endif /* EFI_CJ125 */


	// periodic events need to be initialized after fuel&spark pins to avoid a warning
	initPeriodicEvents(PASS_ENGINE_PARAMETER_SIGNATURE);

	if (hasFirmwareError()) {
		return;
	}

	engineStateBlinkingTask.Start();

	initVrPwm();

#if EFI_PWM_TESTER
	initPwmTester();
#endif /* EFI_PWM_TESTER */

#if EFI_ALTERNATOR_CONTROL
	initAlternatorCtrl();
#endif /* EFI_ALTERNATOR_CONTROL */

#if EFI_AUX_PID
	initAuxPid(sharedLogger);
#endif /* EFI_AUX_PID */

#if EFI_MALFUNCTION_INDICATOR
	initMalfunctionIndicator();
#endif /* EFI_MALFUNCTION_INDICATOR */

	initEgoAveraging();

	if (engineConfiguration->externalKnockSenseAdc != EFI_ADC_NONE) {
		addConsoleAction("knockinfo", getKnockInfo);
	}

#if EFI_PROD_CODE
	addConsoleAction("reset_accel", resetAccel);
#endif /* EFI_PROD_CODE */

#if EFI_HD44780_LCD
	initLcdController();
#endif /* EFI_HD44780_LCD */

}

// these two variables are here only to let us know how much RAM is available, also these
// help to notice when RAM usage goes up - if a code change adds to RAM usage these variables would fail
// linking process which is the way to raise the alarm
#ifndef RAM_UNUSED_SIZE
#define RAM_UNUSED_SIZE 4000
#endif
#ifndef CCM_UNUSED_SIZE
#define CCM_UNUSED_SIZE 2400
#endif
static char UNUSED_RAM_SIZE[RAM_UNUSED_SIZE];
static char UNUSED_CCM_SIZE[CCM_UNUSED_SIZE] CCM_OPTIONAL;

/**
 * See also VCS_VERSION
 */
int getRusEfiVersion(void) {
	if (UNUSED_RAM_SIZE[0] != 0)
		return 123; // this is here to make the compiler happy about the unused array
	if (UNUSED_CCM_SIZE[0] * 0 != 0)
		return 3211; // this is here to make the compiler happy about the unused array
#if defined(EFI_BOOTLOADER_INCLUDE_CODE)
	// make bootloader code happy too
	if (initBootloader() != 0)
		return 123;
#endif /* EFI_BOOTLOADER_INCLUDE_CODE */
	return 20200401;
}
#endif /* EFI_UNIT_TEST */
