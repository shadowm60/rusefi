/*
 * @file test_etb.cpp
 *
 * @date Dec 13, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "engine_test_helper.h"
#include "electronic_throttle_impl.h"
#include "dc_motor.h"
#include "engine_controller.h"
#include "sensor.h"
#include "idle_thread.h"

#include "mocks.h"

using ::testing::_;
using ::testing::Ne;
using ::testing::StrictMock;

TEST(etb, initializationNoPedal) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	EXPECT_CALL(mocks[0], init(ETB_Throttle1, _, _, _, false)).WillOnce(Return(false));
	EXPECT_CALL(mocks[1], init(ETB_Throttle2, _, _, _, false)).WillOnce(Return(false));

	// This shouldn't throw, since no throttles are configured, but no pedal is configured either
	EXPECT_NO_FATAL_ERROR(doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE));
}

TEST(etb, initializationMissingThrottle) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	engineConfiguration->etbFunctions[0] = ETB_None;
	engineConfiguration->etbFunctions[1] = ETB_None;

	EXPECT_CALL(mocks[0], init(ETB_None, _, _, _, true)).WillOnce(Return(false));
	EXPECT_CALL(mocks[1], init(ETB_None, _, _, _, true)).WillOnce(Return(false));

	// Must have a sensor configured before init
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0);
	Sensor::setMockValue(SensorType::AcceleratorPedalPrimary, 0);

	// This should throw: a pedal is configured but no throttles
	EXPECT_FATAL_ERROR(doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE));
}

TEST(etb, initializationSingleThrottle) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	// Must have a sensor configured before init
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0);
	Sensor::setMockValue(SensorType::AcceleratorPedalPrimary, 0);

	engineConfiguration->etbFunctions[0] = ETB_Throttle1;
	engineConfiguration->etbFunctions[1] = ETB_None;

	// Expect mock0 to be init as throttle 1, and PID params
	EXPECT_CALL(mocks[0], init(ETB_Throttle1, _, &engineConfiguration->etb, Ne(nullptr), true)).WillOnce(Return(true));

	// Expect mock1 to be init as none
	EXPECT_CALL(mocks[1], init(ETB_None, _, _, _, true)).WillOnce(Return(true));

	doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE);
}

TEST(etb, initializationSingleThrottleInSecondSlot) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	// Must have a sensor configured before init
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0);
	Sensor::setMockValue(SensorType::AcceleratorPedalPrimary, 0);

	engineConfiguration->etbFunctions[0] = ETB_None;
	engineConfiguration->etbFunctions[1] = ETB_Throttle1;

	// Expect mock0 to be init as none
	EXPECT_CALL(mocks[0], init(ETB_None, _, _, _, true)).WillOnce(Return(true));

	// Expect mock1 to be init as throttle 1, and PID params
	EXPECT_CALL(mocks[1], init(ETB_Throttle1, _, &engineConfiguration->etb, Ne(nullptr), true)).WillOnce(Return(true));

	doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE);
}

TEST(etb, initializationDualThrottle) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	// Must have a sensor configured before init
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0);
	Sensor::setMockValue(SensorType::AcceleratorPedalPrimary, 0);

	Sensor::setMockValue(SensorType::Tps1, 25.0f, true);
	Sensor::setMockValue(SensorType::Tps2, 25.0f, true);

	engineConfiguration->etbFunctions[0] = ETB_Throttle1;
	engineConfiguration->etbFunctions[1] = ETB_Throttle2;

	// Expect mock0 to be init as throttle 1, and PID params
	EXPECT_CALL(mocks[0], init(ETB_Throttle1, _, &engineConfiguration->etb, Ne(nullptr), true)).WillOnce(Return(true));

	// Expect mock1 to be init as throttle 2, and PID params
	EXPECT_CALL(mocks[1], init(ETB_Throttle2, _, &engineConfiguration->etb, Ne(nullptr), true)).WillOnce(Return(true));

	doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE);
}

TEST(etb, initializationWastegate) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];
	}

	engineConfiguration->etbFunctions[0] = ETB_Wastegate;
	engineConfiguration->etbFunctions[1] = ETB_None;

	// Expect mock0 to be init as throttle 1, and PID wastegate params
	EXPECT_CALL(mocks[0], init(ETB_Wastegate, _, &engineConfiguration->etbWastegatePid, Ne(nullptr), false)).WillOnce(Return(true));

	// Expect mock1 to be init as none
	EXPECT_CALL(mocks[1], init(ETB_None, _, _, _, false)).WillOnce(Return(true));

	doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE);
}

TEST(etb, initializationNoFunction) {
	StrictMock<MockMotor> motor;

	EtbController dut;

	// When init called with ETB_None, should ignore the provided params and return false
	EXPECT_FALSE(dut.init(ETB_None, &motor, nullptr, nullptr, false));

	// This should no-op, it shouldn't call motor.set(float duty)
	dut.setOutput(0.5f);
}

TEST(etb, initializationNotRedundantTps) {
	EtbController dut;

	// Needs pedal for init
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);

	// Not redundant, should fail upon init
	Sensor::setMockValue(SensorType::Tps1, 0.0f, false);

	EXPECT_FATAL_ERROR(dut.init(ETB_Throttle1, nullptr, nullptr, nullptr, true));
}

TEST(etb, initializationNoThrottles) {
	// This tests the case where you don't want an ETB, and expect everything to go fine
	EtbController duts[2];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &duts[i];
	}

	// Configure ETB functions, but no pedal
	engineConfiguration->etbFunctions[0] = ETB_Throttle1;
	engineConfiguration->etbFunctions[1] = ETB_Throttle2;

	// No pedal - don't init ETBs
	Sensor::resetMockValue(SensorType::AcceleratorPedal);

	// Not redundant TPS (aka cable throttle)
	Sensor::setMockValue(SensorType::Tps1, 0.0f, false);

	EXPECT_NO_FATAL_ERROR(doInitElectronicThrottle(PASS_ENGINE_PARAMETER_SIGNATURE));
}

TEST(etb, idlePlumbing) {
	StrictMock<MockEtb> mocks[ETB_COUNT];

	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	engineConfiguration->useETBforIdleControl = true;

	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);

	for (int i = 0; i < ETB_COUNT; i++) {
		engine->etbControllers[i] = &mocks[i];

		EXPECT_CALL(mocks[i], setIdlePosition(33.0f));
	}

	applyIACposition(33.0f PASS_ENGINE_PARAMETER_SUFFIX);
}

TEST(etb, testSetpointOnlyPedal) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// Don't use ETB for idle, we aren't testing that yet - just pedal table for now
	engineConfiguration->useETBforIdleControl = false;

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);

	// Mock pedal map that's just passthru pedal -> target
	StrictMock<MockVp3d> pedalMap;
	EXPECT_CALL(pedalMap, getValue(_, _))
		.WillRepeatedly([](float xRpm, float y) {
			return y;
		});

	// Uninitialized ETB must return unexpected (and not deference a null pointer)
	EXPECT_EQ(etb.getSetpoint(), unexpected);

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	etb.init(ETB_Throttle1, nullptr, nullptr, &pedalMap, true);

	// Check endpoints and midpoint
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);
	EXPECT_EQ(0, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);
	EXPECT_EQ(50, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 100.0f);
	EXPECT_EQ(100, etb.getSetpoint().value_or(-1));

	// Test some floating point pedal/output values
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.8302f);
	EXPECT_NEAR(50.8302, etb.getSetpoint().value_or(-1), EPS4D);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 51.6605f);
	EXPECT_NEAR(51.6605, etb.getSetpoint().value_or(-1), EPS4D);

	// Valid but out of range - should clamp to [0, 100]
	Sensor::setMockValue(SensorType::AcceleratorPedal, -5);
	EXPECT_EQ(0, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 105);
	EXPECT_EQ(100, etb.getSetpoint().value_or(-1));

	// Check that ETB idle does NOT work - it's disabled
	etb.setIdlePosition(50);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0);
	EXPECT_EQ(0, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 20);
	EXPECT_EQ(20, etb.getSetpoint().value_or(-1));

	// Test invalid pedal position - should give 0 position
	Sensor::resetMockValue(SensorType::AcceleratorPedal);
	EXPECT_EQ(0, etb.getSetpoint().value_or(-1));
}

TEST(etb, setpointIdle) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// Use ETB for idle, but don't give it any range (yet)
	engineConfiguration->useETBforIdleControl = true;
	engineConfiguration->etbIdleThrottleRange = 0;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);

	// Mock pedal map that's just passthru pedal -> target
	StrictMock<MockVp3d> pedalMap;
	EXPECT_CALL(pedalMap, getValue(_, _))
		.WillRepeatedly([](float xRpm, float y) {
			return y;
		});
	etb.init(ETB_Throttle1, nullptr, nullptr, &pedalMap, true);

	// No idle range, should just pass pedal
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);
	EXPECT_EQ(0, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);
	EXPECT_EQ(50, etb.getSetpoint().value_or(-1));

	// Idle should now have 10% range
	engineConfiguration->etbIdleThrottleRange = 10;

	// 50% idle position should increase setpoint by 5% when closed, and 0% when open
	etb.setIdlePosition(50);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);
	EXPECT_FLOAT_EQ(5, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);
	EXPECT_FLOAT_EQ(52.5, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 100.0f);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));

	// 100% setpoint should increase by 10% closed, scaled 0% at wot
	etb.setIdlePosition(100);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);
	EXPECT_FLOAT_EQ(10, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);
	EXPECT_FLOAT_EQ(55, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 100.0f);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));

	// 125% setpoint should clamp to 10% increase
	etb.setIdlePosition(125);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 0.0f);
	EXPECT_FLOAT_EQ(10, etb.getSetpoint().value_or(-1));
	Sensor::setMockValue(SensorType::AcceleratorPedal, 50.0f);
	EXPECT_FLOAT_EQ(55, etb.getSetpoint().value_or(-1));
}

TEST(etb, setpointNoPedalMap) {
	EtbController etb;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	// Don't pass a pedal map
	etb.init(ETB_Throttle1, nullptr, nullptr, nullptr, true);

	EXPECT_EQ(etb.getSetpoint(), unexpected);
}

TEST(etb, setpointIdleValveController) {
	EtbController etb;

	etb.init(ETB_IdleValve, nullptr, nullptr, nullptr, false);

	etb.setIdlePosition(0);
	EXPECT_FLOAT_EQ(0, etb.getSetpoint().value_or(-1));
	etb.setIdlePosition(50);
	EXPECT_FLOAT_EQ(50, etb.getSetpoint().value_or(-1));
	etb.setIdlePosition(100);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));

	// Out of range should be clamped
	etb.setIdlePosition(-10);
	EXPECT_FLOAT_EQ(0, etb.getSetpoint().value_or(-1));
	etb.setIdlePosition(110);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));
}

TEST(etb, setpointWastegateController) {
	EtbController etb;

	etb.init(ETB_Wastegate, nullptr, nullptr, nullptr, false);

	etb.setWastegatePosition(0);
	EXPECT_FLOAT_EQ(0, etb.getSetpoint().value_or(-1));
	etb.setWastegatePosition(50);
	EXPECT_FLOAT_EQ(50, etb.getSetpoint().value_or(-1));
	etb.setWastegatePosition(100);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));

	// Out of range should be clamped
	etb.setWastegatePosition(-10);
	EXPECT_FLOAT_EQ(0, etb.getSetpoint().value_or(-1));
	etb.setWastegatePosition(110);
	EXPECT_FLOAT_EQ(100, etb.getSetpoint().value_or(-1));
}

TEST(etb, etbTpsSensor) {
	// Throw some distinct values on the TPS sensors so we can identify that we're getting the correct one
	Sensor::setMockValue(SensorType::Tps1, 25.0f, true);
	Sensor::setMockValue(SensorType::Tps2, 75.0f, true);
	Sensor::setMockValue(SensorType::WastegatePosition, 33.0f);
	Sensor::setMockValue(SensorType::IdlePosition, 66.0f);

	// Test first throttle
	{
		EtbController etb;
		etb.init(ETB_Throttle1, nullptr, nullptr, nullptr, true);
		EXPECT_EQ(etb.observePlant().Value, 25.0f);
	}

	// Test second throttle
	{
		EtbController etb;
		etb.init(ETB_Throttle2, nullptr, nullptr, nullptr, true);
		EXPECT_EQ(etb.observePlant().Value, 75.0f);
	}

	// Test wastegate control
	{
		EtbController etb;
		etb.init(ETB_Wastegate, nullptr, nullptr, nullptr, true);
		EXPECT_EQ(etb.observePlant().Value, 33.0f);
	}

	// Test idle valve control
	{
		EtbController etb;
		etb.init(ETB_IdleValve, nullptr, nullptr, nullptr, true);
		EXPECT_EQ(etb.observePlant().Value, 66.0f);
	}
}

TEST(etb, setOutputInvalid) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	StrictMock<MockMotor> motor;

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Should be disabled in case of unexpected
	EXPECT_CALL(motor, disable());

	etb.setOutput(unexpected);
}

TEST(etb, setOutputValid) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	StrictMock<MockMotor> motor;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Should be enabled and value set
	EXPECT_CALL(motor, enable());
	EXPECT_CALL(motor, set(0.25f))
		.WillOnce(Return(false));

	etb.setOutput(25.0f);
}

TEST(etb, setOutputValid2) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	StrictMock<MockMotor> motor;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Should be enabled and value set
	EXPECT_CALL(motor, enable());
	EXPECT_CALL(motor, set(-0.25f))
		.WillOnce(Return(false));

	etb.setOutput(-25.0f);
}

TEST(etb, setOutputOutOfRangeHigh) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	StrictMock<MockMotor> motor;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Should be enabled and value set
	EXPECT_CALL(motor, enable());
	EXPECT_CALL(motor, set(0.90f));

	// Off scale - should get clamped to 90%
	etb.setOutput(110);
}

TEST(etb, setOutputOutOfRangeLow) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	StrictMock<MockMotor> motor;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Should be enabled and value set
	EXPECT_CALL(motor, enable());
	EXPECT_CALL(motor, set(-0.90f));

	// Off scale - should get clamped to -90%
	etb.setOutput(-110);
}

TEST(etb, setOutputPauseControl) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	StrictMock<MockMotor> motor;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, &motor, nullptr, nullptr, true);

	// Pause control - should get no output
	engineConfiguration->pauseEtbControl = true;

	// Disable should be called, and set shouldn't be called
	EXPECT_CALL(motor, disable());

	etb.setOutput(25.0f);
}

TEST(etb, closedLoopPid) {
	pid_s pid = {};
	pid.pFactor = 5;
	pid.maxValue = 75;
	pid.minValue = -60;

	// Must have TPS initialized for ETB setup
	Sensor::setMockValue(SensorType::Tps1, 0.0f, true);

	EtbController etb;
	etb.init(ETB_Throttle1, nullptr, &pid, nullptr, true);

	// Disable autotune for now
	Engine e;
	e.etbAutoTune = false;
	etb.engine = &e;

	// Setpoint greater than actual, should be positive output
	EXPECT_FLOAT_EQ(etb.getClosedLoop(50, 40).value_or(-1), 50);
	// Likewise but negative
	EXPECT_FLOAT_EQ(etb.getClosedLoop(50, 52).value_or(-1), -10);

	// Test PID limiting
	EXPECT_FLOAT_EQ(etb.getClosedLoop(50, 70).value_or(-1), -60);
	EXPECT_FLOAT_EQ(etb.getClosedLoop(50, 30).value_or(-1), 75);
}

TEST(etb, openLoopThrottle) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Throttle1, nullptr, nullptr, nullptr, true);

	// Map [0, 100] -> [-50, 50]
	setLinearCurve(engineConfiguration->etbBiasBins, 0, 100);
	setLinearCurve(engineConfiguration->etbBiasValues, -50, 50);

	EXPECT_NEAR(-50, etb.getOpenLoop(0).value_or(-1), EPS4D);
	EXPECT_NEAR(-25, etb.getOpenLoop(25).value_or(-1), EPS4D);
	EXPECT_NEAR(0, etb.getOpenLoop(50).value_or(-1), EPS4D);
	EXPECT_NEAR(25, etb.getOpenLoop(75).value_or(-1), EPS4D);
	EXPECT_NEAR(50, etb.getOpenLoop(100).value_or(-1), EPS4D);
}

TEST(etb, openLoopNonThrottle) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	EtbController etb;
	INJECT_ENGINE_REFERENCE(&etb);
	etb.init(ETB_Wastegate, nullptr, nullptr, nullptr, false);

	// Map [0, 100] -> [-50, 50]
	setLinearCurve(engineConfiguration->etbBiasBins, 0, 100);
	setLinearCurve(engineConfiguration->etbBiasValues, -50, 50);

	// Should all return 0, as non-throttles don't use open loop table
	EXPECT_EQ(0, etb.getOpenLoop(0).value_or(-1));
	EXPECT_EQ(0, etb.getOpenLoop(25).value_or(-1));
	EXPECT_EQ(0, etb.getOpenLoop(50).value_or(-1));
	EXPECT_EQ(0, etb.getOpenLoop(75).value_or(-1));
	EXPECT_EQ(0, etb.getOpenLoop(100).value_or(-1));
}
