#include "engine_test_helper.h"
#include "vvt.h"
#include "mocks.h"

using ::testing::StrictMock;
using ::testing::Return;

TEST(Vvt, setpoint) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	// Set up a mock target map
	StrictMock<MockVp3d> targetMap;
	EXPECT_CALL(targetMap, getValue(4321, 55))
		.WillOnce(Return(20));

	// Mock necessary inputs
	engine->engineState.fuelingLoad = 55;
	engine->rpmCalculator.mockRpm = 4321;

	VvtController dut;
	INJECT_ENGINE_REFERENCE(&dut);
	dut.init(0, 0, 0, &targetMap);

	// Test dut
	EXPECT_EQ(20, dut.getSetpoint().value_or(0));
}

TEST(Vvt, observePlant) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	engine->triggerCentral.vvtPosition[0][0] = 23;

	VvtController dut;
	INJECT_ENGINE_REFERENCE(&dut);
	dut.init(0, 0, 0, nullptr);

	EXPECT_EQ(23, dut.observePlant().value_or(0));
}

TEST(Vvt, openLoop) {
	VvtController dut;

	// No open loop for now
	EXPECT_EQ(dut.getOpenLoop(10), 0);
}
