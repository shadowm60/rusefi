#include "map.h"
#include "adc_inputs.h"
#include "function_pointer_sensor.h"
#include "engine.h"

EXTERN_ENGINE;

struct GetMapWrapper {
	DECLARE_ENGINE_PTR;

	float getMap() {
		return ::getMap(PASS_ENGINE_PARAMETER_SIGNATURE);
	}
};

static GetMapWrapper mapWrapper;

static FunctionPointerSensor mapSensor(SensorType::Map,
[]() {
	return mapWrapper.getMap();
});

struct GetBaroWrapper {
	DECLARE_ENGINE_PTR;

	float getBaro() {
		return ::getBaroPressure(PASS_ENGINE_PARAMETER_SIGNATURE);
	}
};

static GetBaroWrapper baroWrapper;

static FunctionPointerSensor baroSensor(SensorType::BarometricPressure,
[]() {
	return baroWrapper.getBaro();
});

void initMap(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	INJECT_ENGINE_REFERENCE(&mapWrapper);
	INJECT_ENGINE_REFERENCE(&baroWrapper);
	mapSensor.Register();

	// Only register if configured
	if (isAdcChannelValid(engineConfiguration->baroSensor.hwChannel)) {
		baroSensor.Register();
	}
}
