#include "unit_test_framework.h"
#include "init.h"
#include "sensor.h"
#include "functional_sensor.h"

#include "engine_test_helper.h"
#include <gtest/gtest.h>

static void postToFuncSensor(Sensor* s, float value) {
	static_cast<FunctionalSensor*>(s)->postRawValue(value, getTimeNowNt());
}

#define EXPECT_POINT_VALID(s, raw, expect) \
	{\
		postToFuncSensor(s, raw); \
		auto res = s->get(); \
		EXPECT_TRUE(res.Valid); \
		EXPECT_NEAR(res.Value, expect, EPS2D); \
	}

#define EXPECT_POINT_INVALID(s, raw) \
	{\
		postToFuncSensor(s, raw); \
		auto res = s->get(); \
		EXPECT_FALSE(res.Valid); \
	}

TEST(SensorInit, Tps) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	CONFIG(tpsMin) = 200;	// 1 volt
	CONFIG(tpsMax) = 800;	// 4 volts

	initTps(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure the sensors were registered
	auto s = const_cast<Sensor*>(Sensor::getSensorOfType(SensorType::Tps1Primary));
	ASSERT_NE(nullptr, s);

	// Test in range
	EXPECT_POINT_VALID(s, 1.0f, 0.0f);	// closed throttle
	EXPECT_POINT_VALID(s, 2.5f, 50.0f);	// half throttle
	EXPECT_POINT_VALID(s, 4.0f, 100.0f) // full throttle

	// Test out of range
	EXPECT_POINT_INVALID(s, 0.0f);
	EXPECT_POINT_INVALID(s, 5.0f);

	// Test that the passthru (redundant sensor) is working
	EXPECT_POINT_VALID(s, 2.5f, 50.0f);
	EXPECT_NEAR(50.0f, Sensor::get(SensorType::Tps1).value_or(-1), EPS2D);
}

TEST(SensorInit, TpsValuesTooClose) {
	/*
	 * todo: fix this, this fails HW CI at the moment
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// Should fail, 0.49 volts apart
	CONFIG(tpsMin) = 200;	// 1.00 volt
	CONFIG(tpsMax) = 298;	// 1.49 volts
	EXPECT_FATAL_ERROR(initTps(PASS_CONFIG_PARAMETER_SIGNATURE));
	Sensor::resetRegistry();

	// Should fail, -0.49 volts apart
	CONFIG(tpsMin) = 298;	// 1.49 volt
	CONFIG(tpsMax) = 200;	// 1.00 volts
	EXPECT_FATAL_ERROR(initTps(PASS_CONFIG_PARAMETER_SIGNATURE));
	Sensor::resetRegistry();

	// Should succeed, 0.51 volts apart
	CONFIG(tpsMin) = 200;	// 1.00 volt
	CONFIG(tpsMax) = 302;	// 1.51 volts
	EXPECT_NO_FATAL_ERROR(initTps(PASS_CONFIG_PARAMETER_SIGNATURE));
	Sensor::resetRegistry();

	// Should succeed, -0.51 volts apart
	CONFIG(tpsMin) = 302;	// 1.51 volt
	CONFIG(tpsMax) = 200;	// 1.00 volts
	EXPECT_NO_FATAL_ERROR(initTps(PASS_CONFIG_PARAMETER_SIGNATURE));
	Sensor::resetRegistry();

	// With no pin, it should be ok that they are the same
	// Should succeed, -0.51 volts apart
	CONFIG(tps1_1AdcChannel) = EFI_ADC_NONE;
	CONFIG(tpsMin) = 200;	// 1.00 volt
	CONFIG(tpsMax) = 200;	// 1.00 volts
	EXPECT_NO_FATAL_ERROR(initTps(PASS_CONFIG_PARAMETER_SIGNATURE));
	Sensor::resetRegistry();
	*/
}

TEST(SensorInit, Pedal) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	CONFIG(throttlePedalPositionAdcChannel) = EFI_ADC_0;
	CONFIG(throttlePedalUpVoltage) = 1;
	CONFIG(throttlePedalWOTVoltage) = 4;

	initTps(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure the sensors were registered
	auto s = const_cast<Sensor*>(Sensor::getSensorOfType(SensorType::AcceleratorPedalPrimary));
	ASSERT_NE(nullptr, s);

	// Test in range
	EXPECT_POINT_VALID(s, 1.0f, 0.0f);	// closed throttle
	EXPECT_POINT_VALID(s, 2.5f, 50.0f);	// half throttle
	EXPECT_POINT_VALID(s, 4.0f, 100.0f) // full throttle

	// Test out of range
	EXPECT_POINT_INVALID(s, 0.0f);
	EXPECT_POINT_INVALID(s, 5.0f);

	// Test that the passthru (redundant sensor) is working
	EXPECT_POINT_VALID(s, 2.5f, 50.0f);
	EXPECT_NEAR(50.0f, Sensor::get(SensorType::AcceleratorPedal).value_or(-1), EPS2D);
}

TEST(SensorInit, DriverIntentNoPedal) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// We have no pedal - so we should get the TPS
	CONFIG(throttlePedalPositionAdcChannel) = EFI_ADC_NONE;

	initTps(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure a sensor got set
	ASSERT_TRUE(Sensor::hasSensor(SensorType::DriverThrottleIntent));

	// Set values so we can identify which one got proxied
	Sensor::setMockValue(SensorType::Tps1, 25);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 75);

	// Should get the TPS
	EXPECT_EQ(Sensor::get(SensorType::DriverThrottleIntent).Value, 25);
}


TEST(SensorInit, DriverIntentWithPedal) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// We have a pedal, so we should get it
	CONFIG(throttlePedalPositionAdcChannel) = EFI_ADC_0;

	initTps(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure a sensor got set
	ASSERT_TRUE(Sensor::hasSensor(SensorType::DriverThrottleIntent));

	// Set values so we can identify which one got proxied
	Sensor::setMockValue(SensorType::Tps1, 25);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 75);

	// Should get the pedal
	EXPECT_EQ(Sensor::get(SensorType::DriverThrottleIntent).Value, 75);
}

TEST(SensorInit, OilPressure) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	CONFIG(oilPressure.hwChannel) = EFI_ADC_0;
	CONFIG(oilPressure.v1) = 1;
	CONFIG(oilPressure.v2) = 4;
	CONFIG(oilPressure.value1) = 0;
	CONFIG(oilPressure.value2) = 1000;

	initOilPressure(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure the sensors were registered
	auto s = const_cast<Sensor*>(Sensor::getSensorOfType(SensorType::OilPressure));
	ASSERT_NE(nullptr, s);

	// Test in range
	EXPECT_POINT_VALID(s, 1.0f, 0.0f);	// minimum
	EXPECT_POINT_VALID(s, 2.5f, 500.0f);	// mid
	EXPECT_POINT_VALID(s, 4.0f, 1000.0f) // maximium

	// Test out of range
	EXPECT_POINT_INVALID(s, 0.0f);
	EXPECT_POINT_INVALID(s, 5.0f);
}

TEST(SensorInit, Clt) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// 2003 neon sensor
	CONFIG(clt.config) = {0, 30, 100, 32500, 7550, 700, 2700};

	initThermistors(PASS_CONFIG_PARAMETER_SIGNATURE);

	// Ensure the sensors were registered
	auto s = const_cast<Sensor*>(Sensor::getSensorOfType(SensorType::Clt));
	ASSERT_NE(nullptr, s);

	// Test in range
	EXPECT_POINT_VALID(s, 4.61648f, 0.0f);	// minimum - 0C
	EXPECT_POINT_VALID(s, 3.6829f, 30.0f);	// mid - 30C
	EXPECT_POINT_VALID(s, 1.0294f, 100.0f)	// maximium - 100C

	// Test out of range
	EXPECT_POINT_INVALID(s, 0.0f);
	EXPECT_POINT_INVALID(s, 5.0f);
}

TEST(SensorInit, Lambda) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	initLambda(PASS_ENGINE_PARAMETER_SIGNATURE);

	auto s = Sensor::getSensorOfType(SensorType::Lambda);
	ASSERT_NE(nullptr, s);
}
