#include "global.h"
#include "adc_inputs.h"
#include "adc_subscription.h"
#include "engine.h"
#include "error_handling.h"
#include "functional_sensor.h"
#include "func_chain.h"
#include "linear_func.h"
#include "resistance_func.h"
#include "thermistor_func.h"

EXTERN_ENGINE;

using resist = ResistanceFunc;
using therm = ThermistorFunc;

// Each one could be either linear or thermistor
struct FuncPair {
	LinearFunc linear;
	FuncChain<resist, therm> thermistor;
};

static CCM_OPTIONAL FunctionalSensor clt(SensorType::Clt, MS2NT(10));
static CCM_OPTIONAL FunctionalSensor iat(SensorType::Iat, MS2NT(10));
static CCM_OPTIONAL FunctionalSensor aux1(SensorType::AuxTemp1, MS2NT(10));
static CCM_OPTIONAL FunctionalSensor aux2(SensorType::AuxTemp2, MS2NT(10));

static FuncPair fclt, fiat, faux1, faux2;

void validateThermistorConfig(thermistor_conf_s& cfg) {
	if (
		cfg.tempC_1 >= cfg.tempC_2 ||
		cfg.tempC_2 >= cfg.tempC_3 ||
		cfg.resistance_1 < cfg.resistance_2 ||
		cfg.resistance_2 < cfg.resistance_3
	) {
		firmwareError(OBD_Engine_Coolant_Temperature_Circuit_Malfunction, "Invalid thermistor configuration: please check that temperatures & resistances are in the correct order.");
	}
}

static SensorConverter& configureTempSensorFunction(thermistor_conf_s& cfg, FuncPair& p, bool isLinear) {
	if (isLinear) {
		p.linear.configure(cfg.resistance_1, cfg.tempC_1, cfg.resistance_2, cfg.tempC_2, -50, 250);

		return p.linear;
	} else /* sensor is thermistor */ {
		validateThermistorConfig(cfg);

		p.thermistor.get<resist>().configure(5.0f, cfg.bias_resistor);
		p.thermistor.get<therm>().configure(cfg);

		return p.thermistor;
	}
}

void configTherm(FunctionalSensor &sensor,
					FuncPair &p,
					ThermistorConf &config,
					bool isLinear) {
	// Configure the conversion function for this sensor
	sensor.setFunction(configureTempSensorFunction(config.config, p, isLinear));
}

static void configureTempSensor(FunctionalSensor &sensor,
								FuncPair &p,
								ThermistorConf &config,
								bool isLinear) {
	auto channel = config.adcChannel;

	// Only register if we have a sensor
	if (!isAdcChannelValid(channel)) {
		return;
	}

	configTherm(sensor, p, config, isLinear);

	// Register & subscribe
	AdcSubscription::SubscribeSensor(sensor, channel, 2);
	sensor.Register();
}

void initThermistors(DECLARE_CONFIG_PARAMETER_SIGNATURE) {
	if (!CONFIG(consumeObdSensors)) {
		configureTempSensor(clt,
						fclt,
						CONFIG(clt),
						CONFIG(useLinearCltSensor));

		configureTempSensor(iat,
						fiat,
						CONFIG(iat),
						CONFIG(useLinearIatSensor));
	}

	configureTempSensor(aux1,
						faux1,
						CONFIG(auxTempSensor1),
						false);

	configureTempSensor(aux2,
						faux2,
						CONFIG(auxTempSensor2),
						false);
}

void reconfigureThermistors(DECLARE_CONFIG_PARAMETER_SIGNATURE) {
	configTherm(clt,
				fclt,
				CONFIG(clt),
				CONFIG(useLinearCltSensor));

	configTherm(iat,
				fiat,
				CONFIG(iat),
				CONFIG(useLinearIatSensor));

	configTherm(aux1,
				faux1,
				CONFIG(auxTempSensor1),
				false);

	configTherm(aux2,
				faux2,
				CONFIG(auxTempSensor2),
				false);
}
