/**
 * @file	speed_density.cpp
 *
 * See http://rusefi.com/wiki/index.php?title=Manual:Software:Fuel_Control#Speed_Density for details
 *
 * @date May 29, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "global.h"
#include "speed_density.h"
#include "fuel_math.h"
#include "interpolation.h"
#include "engine.h"
#include "engine_math.h"
#include "perf_trace.h"
#include "sensor.h"
#include "map.h"

#if defined(HAS_OS_ACCESS)
#error "Unexpected OS ACCESS HERE"
#endif

#define rpmMin 500
#define rpmMax 8000

EXTERN_ENGINE;

fuel_Map3D_t veMap("VE");
fuel_Map3D_t ve2Map("VE2");
lambda_Map3D_t lambdaMap("lambda");
baroCorr_Map3D_t baroCorrMap("baro");

#define tpMin 0
#define tpMax 100
//  http://rusefi.com/math/t_charge.html
/***panel:Charge Temperature*/
temperature_t getTCharge(int rpm, float tps DECLARE_ENGINE_PARAMETER_SUFFIX) {
	const auto clt = Sensor::get(SensorType::Clt);
	const auto iat = Sensor::get(SensorType::Iat);

	float airTemp = 0;

	// Without either valid, return 0C.  It's wrong, but it'll pretend to be nice and dense, so at least you won't go lean.
	if (!iat && !clt) {
		return 0;
	} else if (!clt && iat) {
		// Intake temperature will almost always be colder (richer) than CLT - use that
		return airTemp;
	} else if (!iat && clt) {
		// Without valid intake temperature, assume intake temp is 0C, and interpolate anyway
		airTemp = 0;
	} else {
		// All is well - use real air temp
		airTemp = iat.Value;
	}

	float coolantTemp = clt.Value;

	DISPLAY_STATE(Engine)

	if ((engine->engineState.sd.DISPLAY_IF(isTChargeAirModel) = (CONFIG(tChargeMode) == TCHARGE_MODE_AIR_INTERP))) {
		const floatms_t gramsPerMsToKgPerHour = (3600.0f * 1000.0f) / 1000.0f;
		// We're actually using an 'old' airMass calculated for the previous cycle, but it's ok, we're not having any self-excitaton issues
		floatms_t airMassForEngine = engine->engineState.sd./***display*/airMassInOneCylinder * CONFIG(specs.cylindersCount);
		// airMass is in grams per 1 cycle for 1 cyl. Convert it to airFlow in kg/h for the engine.
		// And if the engine is stopped (0 rpm), then airFlow is also zero (avoiding NaN division)
		floatms_t airFlow = (rpm == 0) ? 0 : airMassForEngine * gramsPerMsToKgPerHour / getEngineCycleDuration(rpm PASS_ENGINE_PARAMETER_SUFFIX);
		// just interpolate between user-specified min and max coefs, based on the max airFlow value
		DISPLAY_TEXT(interpolate_Air_Flow)
		engine->engineState.DISPLAY_FIELD(airFlow) = airFlow;
		DISPLAY_TEXT(Between)
		engine->engineState.sd.Tcharge_coff = interpolateClamped(0.0,
				CONFIG(DISPLAY_CONFIG(tChargeAirCoefMin)),
				CONFIG(DISPLAY_CONFIG(tChargeAirFlowMax)),
				CONFIG(DISPLAY_CONFIG(tChargeAirCoefMax)), airFlow);
		// save it for console output (instead of MAF massAirFlow)
	} else/* DISPLAY_ELSE */ {
		// TCHARGE_MODE_RPM_TPS
		DISPLAY_TEXT(interpolate_3D)
		DISPLAY_SENSOR(RPM)
		DISPLAY_TEXT(and)
		DISPLAY_SENSOR(TPS)
		DISPLAY_TEXT(EOL)
		DISPLAY_TEXT(Between)
		float minRpmKcurrentTPS = interpolateMsg("minRpm", tpMin,
				CONFIG(DISPLAY_CONFIG(tChargeMinRpmMinTps)), tpMax,
				CONFIG(DISPLAY_CONFIG(tChargeMinRpmMaxTps)), tps);
		DISPLAY_TEXT(EOL)
		float maxRpmKcurrentTPS = interpolateMsg("maxRpm", tpMin,
				CONFIG(DISPLAY_CONFIG(tChargeMaxRpmMinTps)), tpMax,
				CONFIG(DISPLAY_CONFIG(tChargeMaxRpmMaxTps)), tps);

		engine->engineState.sd.Tcharge_coff = interpolateMsg("Kcurr", rpmMin, minRpmKcurrentTPS, rpmMax, maxRpmKcurrentTPS, rpm);
	/* DISPLAY_ENDIF */
	}

	if (cisnan(engine->engineState.sd.Tcharge_coff)) {
		warning(CUSTOM_ERR_T2_CHARGE, "t2-getTCharge NaN");
		return coolantTemp;
	}

	// We use a robust interp. function for proper tcharge_coff clamping.
	float Tcharge = interpolateClamped(0.0f, coolantTemp, 1.0f, airTemp, engine->engineState.sd.Tcharge_coff);

	if (cisnan(Tcharge)) {
		// we can probably end up here while resetting engine state - interpolation would fail
		warning(CUSTOM_ERR_TCHARGE_NOT_READY, "getTCharge NaN");
		return coolantTemp;
	}

	return Tcharge;
}

void setDefaultVETable(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	setRpmTableBin(config->veRpmBins, FUEL_RPM_COUNT);
	veMap.setAll(80);

//	setRpmTableBin(engineConfiguration->ve2RpmBins, FUEL_RPM_COUNT);
//	setLinearCurve(engineConfiguration->ve2LoadBins, 10, 300, 1);
//	ve2Map.setAll(0.81);

	setRpmTableBin(config->lambdaRpmBins, FUEL_RPM_COUNT);
	lambdaMap.setAll(1.0);

	setRpmTableBin(engineConfiguration->baroCorrRpmBins, BARO_CORR_SIZE);
	setLinearCurve(engineConfiguration->baroCorrPressureBins, 75, 105, 1);
	for (int i = 0; i < BARO_CORR_SIZE;i++) {
		for (int j = 0; j < BARO_CORR_SIZE;j++) {
			// Default baro table is all 1.0, we can't recommend a reasonable default here
			engineConfiguration->baroCorrTable[i][j] = 1;
		}
	}
}

void initSpeedDensity(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	veMap.init(config->veTable, config->veLoadBins, config->veRpmBins);
//	ve2Map.init(engineConfiguration->ve2Table, engineConfiguration->ve2LoadBins, engineConfiguration->ve2RpmBins);
	lambdaMap.init(config->lambdaTable, config->lambdaLoadBins, config->lambdaRpmBins);
	baroCorrMap.init(engineConfiguration->baroCorrTable, engineConfiguration->baroCorrPressureBins, engineConfiguration->baroCorrRpmBins);
}
