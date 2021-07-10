#include "global.h"
#include "engine.h"
#include "maf_airmass.h"
#include "maf.h"

EXTERN_ENGINE;

AirmassResult MafAirmass::getAirmass(int rpm) {
	float maf = Sensor::get(SensorType::Maf).value_or(0) + engine->engineLoadAccelEnrichment.getEngineLoadEnrichment(PASS_ENGINE_PARAMETER_SIGNATURE);
	return getAirmassImpl(maf, rpm);
}

/**
 * Function block now works to create a standardised load from the cylinder filling as well as tune fuel via VE table. 
 * @return total duration of fuel injection per engine cycle, in milliseconds
 */
AirmassResult MafAirmass::getAirmassImpl(float massAirFlow, int rpm) const {
	// If the engine is stopped, MAF is meaningless
	if (rpm == 0) {
		return {};
	}

	// kg/hr -> g/s
	float gramPerSecond = massAirFlow * 1000 / 3600;

	// 1/min -> 1/s
	float revsPerSecond = rpm / 60.0f;
	float airPerRevolution = gramPerSecond / revsPerSecond;

	// Now we have to divide among cylinders - on a 4 stroke, half of the cylinders happen every rev
	// This math is floating point to work properly on engines with odd cyl count
	float halfCylCount = CONFIG(specs.cylindersCount) / 2.0f;

	float cylinderAirmass = airPerRevolution / halfCylCount;

	//Create % load for fuel table using relative naturally aspiratedcylinder filling
	float airChargeLoad = 100 * cylinderAirmass / ENGINE(standardAirCharge);
	
	//Correct air mass by VE table
	float correctedAirmass = cylinderAirmass * getVe(rpm, airChargeLoad);

	return {
		correctedAirmass,
		airChargeLoad, // AFR/VE/ignition table Y axis
	};
}
