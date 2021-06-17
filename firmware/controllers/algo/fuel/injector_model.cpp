#include "injector_model.h"
#include "tunerstudio_outputs.h"
#include "map.h"

EXTERN_ENGINE;

void InjectorModelBase::prepare() {
	m_massFlowRate = getInjectorMassFlowRate();
	float deadtime = getDeadtime();
	m_deadtime = deadtime;

	postState(deadtime);
}

constexpr float convertToGramsPerSecond(float ccPerMinute) {
	float ccPerSecond = ccPerMinute / 60;
	return ccPerSecond * 0.72f;	// 0.72g/cc fuel density
}

expected<float> InjectorModel::getAbsoluteRailPressure() const {
	switch (CONFIG(injectorCompensationMode)) {
		case ICM_FixedRailPressure:
			// Add barometric pressure, as "fixed" really means "fixed pressure above atmosphere"
			return CONFIG(fuelReferencePressure) + Sensor::get(SensorType::BarometricPressure).value_or(101.325f);
		case ICM_SensedRailPressure:
			if (!Sensor::hasSensor(SensorType::FuelPressureInjector)) {
				firmwareError(OBD_PCM_Processor_Fault, "Fuel pressure compensation is set to use a pressure sensor, but none is configured.");
				return unexpected;
			}

			// TODO: what happens when the sensor fails?
			return Sensor::get(SensorType::FuelPressureInjector);
		default: return unexpected;
	}
}

float InjectorModel::getInjectorFlowRatio() const {
	// Compensation disabled, use reference flow.
	if (CONFIG(injectorCompensationMode) == ICM_None) {
		return 1.0f;
	}

	float referencePressure = CONFIG(fuelReferencePressure);
	expected<float> absRailPressure = getAbsoluteRailPressure();

	// If sensor failed, best we can do is disable correction
	if (!absRailPressure) {
		return 1.0f;
	}

	auto map = Sensor::get(SensorType::Map);

	// Map has failed, assume nominal pressure
	if (!map) {
		return 1.0f;
	}

	float pressureDelta = absRailPressure.Value - map.Value;

	// Somehow pressure delta is less than 0, assume failed sensor and return default flow
	if (pressureDelta <= 0) {
		return 1.0f;
	}

	float pressureRatio = pressureDelta / referencePressure;
	float flowRatio = sqrtf(pressureRatio);

#if EFI_TUNER_STUDIO
	if (CONFIG(debugMode) == DBG_INJECTOR_COMPENSATION) {
		tsOutputChannels.debugFloatField1 = pressureDelta;
		tsOutputChannels.debugFloatField2 = pressureRatio;
		tsOutputChannels.debugFloatField3 = flowRatio;
	}
#endif // EFI_TUNER_STUDIO

	// TODO: should the flow ratio be clamped?
	return flowRatio;
}

float InjectorModel::getInjectorMassFlowRate() const {
	// TODO: injector flow dependent upon temperature/ethanol content?
	auto injectorVolumeFlow = CONFIG(injector.flow);

	float flowRatio = getInjectorFlowRatio();

	return flowRatio * convertToGramsPerSecond(injectorVolumeFlow);
}

float InjectorModel::getDeadtime() const {
	return interpolate2d(
		Sensor::get(SensorType::BatteryVoltage).value_or(VBAT_FALLBACK_VALUE),
		engineConfiguration->injector.battLagCorrBins,
		engineConfiguration->injector.battLagCorr
	);
}

void InjectorModel::postState(float deadtime) const {
	engine->engineState.running.injectorLag = deadtime;
}

float InjectorModelBase::getInjectionDuration(float fuelMassGram) const {
	// TODO: support injector nonlinearity correction

	floatms_t baseDuration = fuelMassGram / m_massFlowRate * 1000;

	if (baseDuration <= 0) {
		// If 0 duration, don't add deadtime, just skip the injection.
		return 0.0f;
	} else {
		return baseDuration + m_deadtime;
	}
}

float InjectorModelBase::getFuelMassForDuration(floatms_t duration) const {
	// Convert from ms -> grams
	return duration * m_massFlowRate * 0.001f;
}
