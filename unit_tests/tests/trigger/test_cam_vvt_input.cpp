/*
 * test_cam_vvt_input.cpp
 *
 *  Created on: Jan 13, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "engine_test_helper.h"
extern WarningCodeState unitTestWarningCodeState;

TEST(trigger, testNoStartUpWarningsNoSyncronizationTrigger) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	// one tooth does not need synchronization it just counts tooth
	eth.setTriggerType(TT_ONE PASS_ENGINE_PARAMETER_SUFFIX);
	ASSERT_EQ( 0,  GET_RPM()) << "testNoStartUpWarnings RPM";

	eth.fireTriggerEvents2(/*count*/10, /*duration*/50);
	ASSERT_EQ(1200,  GET_RPM()) << "testNoStartUpWarnings RPM";
	ASSERT_EQ( 0,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testNoStartUpWarningsNoSyncronizationTrigger";
}

TEST(trigger, testNoStartUpWarnings) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	// for this test we need a trigger with isSynchronizationNeeded=true
	engineConfiguration->trigger.customTotalToothCount = 3;
	engineConfiguration->trigger.customSkippedToothCount = 1;
	eth.setTriggerType(TT_TOOTHED_WHEEL PASS_ENGINE_PARAMETER_SUFFIX);
	ASSERT_EQ( 0,  GET_RPM()) << "testNoStartUpWarnings RPM";

	for (int i = 0;i < 10;i++) {
		eth.fireRise(50);
		eth.fireFall(50);
		eth.fireRise(50);
		eth.fireFall(150);
	}

	ASSERT_EQ(400,  GET_RPM()) << "testNoStartUpWarnings RPM";
	ASSERT_EQ( 0,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testNoStartUpWarnings";
	// now let's post something unneeded
	eth.fireRise(50);
	eth.fireFall(50);
	eth.fireRise(50); // this is noise
	eth.fireFall(50); // this is noise
	eth.fireRise(50);
	eth.fireFall(150);
	for (int i = 0;i < 1;i++) {
		eth.fireRise(50);
		eth.fireFall(50);
		eth.fireRise(50);
		eth.fireFall(150);
	}
	ASSERT_EQ( 2,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testNoStartUpWarnings CUSTOM_SYNC_COUNT_MISMATCH expected";
	ASSERT_EQ(CUSTOM_SYNC_ERROR, unitTestWarningCodeState.recentWarnings.get(0));
	ASSERT_EQ(CUSTOM_SYNC_COUNT_MISMATCH, unitTestWarningCodeState.recentWarnings.get(1));
}

TEST(trigger, testNoisyInput) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);

	ASSERT_EQ( 0,  GET_RPM()) << "testNoisyInput RPM";

	eth.firePrimaryTriggerRise();
	eth.firePrimaryTriggerFall();
	eth.firePrimaryTriggerRise();
	eth.firePrimaryTriggerFall();
	eth.firePrimaryTriggerRise();
	eth.firePrimaryTriggerFall();
	eth.firePrimaryTriggerRise();
	eth.firePrimaryTriggerFall();
	// error condition since events happened too quick while time does not move
	ASSERT_EQ(NOISY_RPM,  GET_RPM()) << "testNoisyInput RPM should be noisy";

	ASSERT_EQ( 2,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testNoisyInput";
	ASSERT_EQ(CUSTOM_SYNC_COUNT_MISMATCH, unitTestWarningCodeState.recentWarnings.get(0)) << "@0";
	ASSERT_EQ(OBD_Crankshaft_Position_Sensor_A_Circuit_Malfunction, unitTestWarningCodeState.recentWarnings.get(1)) << "@0";
}

TEST(trigger, testCamInput) {
	// setting some weird engine
	WITH_ENGINE_TEST_HELPER(FORD_ESCORT_GT);

	// changing to 'ONE TOOTH' trigger on CRANK with CAM/VVT
	setOperationMode(engineConfiguration, FOUR_STROKE_CRANK_SENSOR);
	engineConfiguration->useOnlyRisingEdgeForTrigger = true;
	engineConfiguration->vvtMode[0] = VVT_FIRST_HALF;
	engineConfiguration->vvtOffset = 720;
	eth.setTriggerType(TT_ONE PASS_ENGINE_PARAMETER_SUFFIX);
	engineConfiguration->camInputs[0] = GPIOA_10; // we just need to indicate that we have CAM

	ASSERT_EQ( 0,  GET_RPM()) << "testCamInput RPM";

	for (int i = 0; i < 5;i++) {
		eth.fireRise(50);
	}

	ASSERT_EQ(1200,  GET_RPM()) << "testCamInput RPM";
	ASSERT_EQ(0,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testCamInput";

	for (int i = 0; i < 600;i++) {
		eth.fireRise(50);
	}

	// asserting that lack of camshaft signal would be detecting
	ASSERT_EQ(1,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testCamInput #2";
	ASSERT_EQ(OBD_Camshaft_Position_Sensor_Circuit_Range_Performance, unitTestWarningCodeState.recentWarnings.get(0)) << "@0";
	unitTestWarningCodeState.recentWarnings.clear();

	for (int i = 0; i < 600;i++) {
		eth.moveTimeForwardUs(MS2US(10));
		hwHandleVvtCamSignal(TV_FALL, getTimeNowNt(), 0 PASS_ENGINE_PARAMETER_SUFFIX);
		eth.moveTimeForwardUs(MS2US(40));
		eth.firePrimaryTriggerRise();
	}

	// asserting that error code has cleared
	ASSERT_EQ(0,  unitTestWarningCodeState.recentWarnings.getCount()) << "warningCounter#testCamInput #3";
	ASSERT_NEAR(720 - 181, engine->triggerCentral.getVVTPosition(), EPS3D);
}

TEST(sensors, testNB2CamInput) {
	WITH_ENGINE_TEST_HELPER(MAZDA_MIATA_2003);

	// this crank trigger would be easier to test, crank shape is less important for this test
	eth.setTriggerType(TT_ONE PASS_ENGINE_PARAMETER_SUFFIX);

	engineConfiguration->useOnlyRisingEdgeForTrigger = true;

	ASSERT_EQ( 0,  GET_RPM()) << "testNB2CamInput RPM";
	for (int i = 0; i < 7;i++) {
		eth.fireRise(25);
		ASSERT_EQ( 0,  GET_RPM()) << "testNB2CamInput RPM";
	}
	eth.fireRise(25);
	// first time we have RPM
	ASSERT_EQ(1200,  GET_RPM()) << "testNB2CamInput RPM";

	int totalRevolutionCountBeforeVvtSync = 10;
	// need to be out of VVT sync to see VVT sync in action
	eth.fireRise(25);
	eth.fireRise(25);
	ASSERT_EQ(totalRevolutionCountBeforeVvtSync, engine->triggerCentral.triggerState.getTotalRevolutionCounter());
	ASSERT_TRUE((totalRevolutionCountBeforeVvtSync % SYMMETRICAL_CRANK_SENSOR_DIVIDER) != 0);

	eth.moveTimeForwardUs(MS2US(3)); // shifting VVT phase a few angles

	// this would be ignored since we only consume the other kind of fronts here
	hwHandleVvtCamSignal(TV_FALL, getTimeNowNt(), 0 PASS_ENGINE_PARAMETER_SUFFIX);
	eth.moveTimeForwardUs(MS2US(20));
	// this would be be first VVT signal - gap duration would be calculated against 'DEEP_IN_THE_PAST_SECONDS' initial value
	hwHandleVvtCamSignal(TV_RISE, getTimeNowNt(), 0 PASS_ENGINE_PARAMETER_SUFFIX);

	eth.moveTimeForwardUs(MS2US(20));
	// this second important front would give us first real VVT gap duration
	hwHandleVvtCamSignal(TV_RISE, getTimeNowNt(), 0 PASS_ENGINE_PARAMETER_SUFFIX);

	ASSERT_FLOAT_EQ(0, engine->triggerCentral.getVVTPosition());
	ASSERT_EQ(totalRevolutionCountBeforeVvtSync, engine->triggerCentral.triggerState.getTotalRevolutionCounter());

	eth.moveTimeForwardUs(MS2US(130));
	// this third important front would give us first comparison between two real gaps
	hwHandleVvtCamSignal(TV_RISE, getTimeNowNt(), 0 PASS_ENGINE_PARAMETER_SUFFIX);

	ASSERT_NEAR(-67.6 - 720 - 720, engine->triggerCentral.getVVTPosition(), EPS3D);
	// actually position based on VVT!
	ASSERT_EQ(totalRevolutionCountBeforeVvtSync + 2, engine->triggerCentral.triggerState.getTotalRevolutionCounter());
}
