/*
 * @file	tunerstudio_outputs.h
 * @brief	Tuner Studio connectivity configuration
 *
 * In this file the configuration of TunerStudio is defined
 *
 * @date Oct 22, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "rusefi_types.h"
#include "scaled_channel.h"
#include "tunerstudio_debug_struct.h"

typedef struct {
	uint16_t values[EGT_CHANNEL_COUNT];
} egt_values_s;


enum class TsCalMode : uint8_t {
	None = 0,
	Tps1Max = 1,
	Tps1Min = 2,
	EtbKp = 3,
	EtbKi = 4,
	EtbKd = 5,
	Tps1SecondaryMax = 6,
	Tps1SecondaryMin = 7,
	Tps2Max = 8,
	Tps2Min = 9,
	Tps2SecondaryMax = 10,
	Tps2SecondaryMin = 11,
};

/**
 * At the moment rusEfi does NOT have any code generation around TS output channels, three locations have to be changed manually
 * 1) this TunerStudioOutputChannels firmware version of the structure
 * 2) '[OutputChannels]' block in rusefi.input
 * 3) com.rusefi.core.Sensor enum in rusEfi console source code
 *
 * please be aware that 'float' (F32) type requires TunerStudio version 2.6 and later
 */
typedef struct {
	/* see also [OutputChannels] in rusefi.input */

	/**
	 * Yes, I do not really enjoy packing bits into integers but we simply have too many boolean flags and I cannot
	 * water 4 bytes per traffic - I want gauges to work as fast as possible
	 */
	unsigned int hasSdCard : 1; // bit 0, 72
	unsigned int isIgnitionEnabled : 1; // bit 1
	unsigned int isInjectionEnabled : 1; // bit 2
	unsigned int isCylinderCleanupEnabled : 1; // bit 3
	unsigned int isCylinderCleanupActivated : 1; // bit 4
	unsigned int isFuelPumpOn : 1; // bit 5
	unsigned int isFanOn : 1; // bit 6
	unsigned int isO2HeaterOn : 1; // bit 7
	unsigned int checkEngine : 1; // bit 8
	unsigned int needBurn : 1; // bit 9
	unsigned int unusedBit10 : 1; // bit 10
	unsigned int clutchUpState : 1; // bit 11
	unsigned int clutchDownState : 1; // bit 12
	unsigned int knockEverIndicator : 1; // bit 13
	unsigned int knockNowIndicator : 1; // bit 14
	unsigned int brakePedalState : 1; // bit 15. 0 - not pressed, 1 = pressed
	unsigned int toothLogReady : 1; // bit 16
	unsigned int acSwitchState : 1; // bit 17. 0 - not pressed, 1 = pressed
	unsigned int isTpsError : 1; // bit 18
	unsigned int isCltError : 1; // bit 19
	unsigned int isMapError : 1; // bit 20
	unsigned int isIatError : 1; // bit 21
	unsigned int unusedAt22 : 1; // bit 22
	unsigned int isTriggerError : 1; // bit 23
	unsigned int hasCriticalError : 1; // bit 24
	unsigned int isWarnNow : 1; // bit 25
	unsigned int isPedalError : 1; // bit 26
	unsigned int isKnockChipOk : 1; // bit 27
	unsigned int launchTriggered : 1; // bit 28
	unsigned int isTps2Error : 1; // bit 29
	unsigned int isIdleClosedLoop : 1; // bit 30

	// RPM, vss
	scaled_channel<uint16_t> rpm;   // 4
	int16_t rpmAcceleration; // 6
	scaled_percent speedToRpmRatio; // 8
	scaled_channel<uint8_t> vehicleSpeedKph; // 10
	
	// temperatures
	scaled_channel<int8_t> internalMcuTemperature; // offset 11
	scaled_temperature coolantTemperature;   // offset 12
	scaled_temperature intakeAirTemperature; // offset 14
	scaled_temperature auxTemp1;             // offset 16
	scaled_temperature auxTemp2;             // offset 18

	// throttle, pedal
	scaled_percent throttlePosition;    // 20
	scaled_percent pedalPosition;       // 22
	uint16_t tpsADC;                    // 24

	// air flow/mass measurment
	scaled_voltage massAirFlowVoltage; // 26
	scaled_channel<uint16_t, PACK_MULT_MASS_FLOW> massAirFlow; // 28
	scaled_pressure manifoldAirPressure; // 30
	scaled_pressure baroPressure; // 32

	scaled_lambda lambda; // 34
	scaled_channel<uint16_t, 100> engineLoad; // 36

	// misc sensors
	scaled_voltage vBatt; // 38
	scaled_pressure oilPressure; // 40
	scaled_angle vvtPosition; // 42

	// Fuel math
	scaled_channel<uint16_t, 1000> chargeAirMass; // 44  cylinder airmass in mg, 0-65 grams
	scaled_fuel_mass_mg crankingFuelMass; // 46
	scaled_afr currentTargetAfr; // 48
	// This is the raw value we take from the fuel map or base fuel algorithm, before the corrections
	scaled_fuel_mass_mg fuelBase; // 50
	// Total fuel with CLT, IAT and TPS acceleration without injector lag corrections per cycle, as pulse per cycle
	scaled_fuel_mass_mg fuelRunning; // 52
	// Actual last injection time - including all compensation and injection mode
	scaled_ms actualLastInjection; // 54
	scaled_channel<uint8_t, 2> injectorDutyCycle; // 56
	scaled_channel<uint8_t, 2> veValue; // 57
	scaled_angle injectionOffset; // 58
	scaled_temperature tCharge; // 60

	// Corrections
	scaled_ms injectorLagMs; // 62
	scaled_percent iatCorrection; // 64
	scaled_percent cltCorrection; // 66
	scaled_percent baroCorrection; // 68
	scaled_percent shortTermFuelTrim; // 70

	// Wall model AE
	scaled_ms wallFuelAmount; // 72
	scaled_channel<int16_t, 1000> wallFuelCorrection; // 74
	
	// TPS/load AE
	scaled_percent engineLoadDelta; // 76
	scaled_percent deltaTps; // 78
	scaled_percent engineLoadAccelExtra; // 80
	scaled_ms tpsAccelFuel; // 82

	// Ignition
	scaled_angle ignitionAdvance; // 84
	scaled_ms sparkDwell; // 86
	scaled_percent coilDutyCycle; // 88

	// Idle & ETB
	scaled_percent idlePosition; // 90
	scaled_percent etbTarget; // 92
	scaled_percent etb1DutyCycle; // 94
	scaled_percent etb1Error; // 96

	// Fuel system
	scaled_percent fuelTankLevel; // 98
	float fuelConsumptionPerHour; // 100

	// Y axis values for selectable tables
	scaled_channel<uint16_t, 100> veTableYAxis;  // 104
	scaled_channel<uint16_t, 100> afrTableYAxis; // 106

	// Knock
	float knockLevel; // 108

	// Mode, firmware, protocol, run time
	uint32_t timeSeconds; // 112
	uint32_t engineMode; // 116
	uint32_t firmwareVersion; // 120
	// todo: this not needed in light of TS_SIGNATURE but rusEFI console still uses it. Need to migrate
	// rusEFI console from TS_FILE_VERSION to TS_SIGNATURE :(

	uint32_t tsConfigVersion; // 124

	// These two fields indicate to TS that we'd like to set a particular field to a particular value
	// We use a maintainConstantValue in TS for each field we'd like to set, like this:
	//		maintainConstantValue = tpsMax, { (calibrationMode == 1 ) ? calibrationValue : tpsMax }
	//		maintainConstantValue = tpsMin, { (calibrationMode == 2 ) ? calibrationValue : tpsMin }
	// When the mode is set to a particular value, TS will copy the calibrationValue in to the specified field.
	//
	// With this simple construct, the ECU can send any number of internally computed configuration fields
	// back to TunerStudio, getting around the problem of setting values on the controller without TS's knowledge.
	// The ECU simply has to sequentially set a mode/value, wait briefly, then repeat until all the values
	// it wants to send have been sent.
	float calibrationValue;	// 128
	TsCalMode calibrationMode; // 132
	uint8_t padding[1]; // 133

	// Values used for load axes for fuel/ign tables
	// These may or may not be the same value, depending on mode
	scaled_channel<uint16_t, 100> fuelingLoad; // 134
	scaled_channel<uint16_t, 100> ignitionLoad; // 136

	// we want a hash of engineMake+engineCode+vehicleName in the log file in order to match TS logs to rusEFI Online tune
	int16_t engineMakeCodeNameCrc16; // 138
	// Errors
	scaled_channel<uint32_t> totalTriggerErrorCounter; // 140
	int orderingErrorCounter; // 144
	int16_t warningCounter; // 148
	int16_t lastErrorCode; // 150
	int16_t recentErrorCodes[8]; // 152-166

	// Debug
	float debugFloatField1; // 168
	float debugFloatField2;
	float debugFloatField3;
	float debugFloatField4;
	float debugFloatField5;
	float debugFloatField6;
	float debugFloatField7;
	int debugIntField1;
	int debugIntField2;
	int debugIntField3;
	int16_t debugIntField4;
	int16_t debugIntField5; // 210

	// accelerometer
	int16_t accelerationX; // 212
	int16_t accelerationY; // 214

	// EGT
	egt_values_s egtValues; // 216

	scaled_percent throttle2Position;    // 232

	scaled_voltage rawTps1Primary;		// 234
	scaled_voltage rawPpsPrimary;		// 236
	scaled_voltage rawClt;				// 238
	scaled_voltage rawIat;				// 240
	scaled_voltage rawOilPressure;		// 242

	int16_t tuneCrc16; // 244

	uint8_t sd_status; // 246

	int8_t tcuCurrentGear; // 247

	scaled_voltage rawPpsSecondary;		// 248

	int8_t knockLevels[12];		// 250

	int8_t tcuDesiredGear; // 262
	int8_t padding2[1];		// 263

	scaled_voltage rawIdlePositionSensor;	// 264
	scaled_voltage rawWastegatePositionSensor;	// 266

	scaled_percent wastegatePosition;	// 268
	scaled_percent idlePositionSensor;	// 270

	scaled_voltage rawLowFuelPressure; // 272
	scaled_voltage rawHighFuelPressure; // 274

	scaled_pressure lowFuelPressure;	// 276
	scaled_high_pressure highFuelPressure;	// 278

	scaled_lambda targetLambda; // 280
	scaled_afr airFuelRatio; // 282

	scaled_ms VssAcceleration; //284

	scaled_lambda lambda2; // 286
	scaled_afr airFuelRatio2; // 288

	//288
	uint8_t unusedAtTheEnd[48]; // we have some unused bytes to allow compatible TS changes

	// Temporary - will remove soon
	TsDebugChannels* getDebugChannels() {
		return reinterpret_cast<TsDebugChannels*>(&debugFloatField1);
	}

	/* see also [OutputChannels] in rusefi.input */
	/* see also TS_OUTPUT_SIZE in rusefi_config.txt */

} TunerStudioOutputChannels;

extern TunerStudioOutputChannels tsOutputChannels;
