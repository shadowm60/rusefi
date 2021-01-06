#include "init.h"
#include "engine.h"
#include "flex_sensor.h"

EXTERN_ENGINE;

static FlexFuelSensor flexSensor;

// https://rusefi.com/forum/viewtopic.php?p=37452&sid=829804c90d5b2e1fecd1b900cf1b1811#p37452

void initFlexSensor(DECLARE_CONFIG_PARAMETER_SIGNATURE) {
	auto pin = CONFIG(flexSensorPin);

	// Nothing to do if no sensor configured
	if (pin == GPIO_UNASSIGNED) {
		return;
	}

	flexSensor.init(pin);
	flexSensor.Register();
}
