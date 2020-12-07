#pragma once

#include "electronic_throttle.h"
#include "dc_motor.h"
#include "table_helper.h"
#include "pwm_generator_logic.h"
#include "airmass.h"

#include "gmock/gmock.h"

class MockEtb : public IEtbController {
public:
	// IEtbController mocks
	MOCK_METHOD(void, reset, (), ());
	MOCK_METHOD(void, update, (), (override));
	MOCK_METHOD(bool, init, (etb_function_e function, DcMotor* motor, pid_s* pidParameters, const ValueProvider3D* pedalMap), (override));
	MOCK_METHOD(void, setIdlePosition, (percent_t pos), (override));
	MOCK_METHOD(void, setWastegatePosition, (percent_t pos), (override));
	MOCK_METHOD(void, autoCalibrateTps, (), (override));
	MOCK_METHOD(const pid_state_s*, getPidState, (), (const, override));

	// ClosedLoopController mocks
	MOCK_METHOD(expected<percent_t>, getSetpoint, (), (const, override));
	MOCK_METHOD(expected<percent_t>, observePlant, (), (const, override));
	MOCK_METHOD(expected<percent_t>, getOpenLoop, (percent_t setpoint), (const, override));
	MOCK_METHOD(expected<percent_t>, getClosedLoop, (percent_t setpoint, percent_t observation), (override));
	MOCK_METHOD(void, setOutput, (expected<percent_t> outputValue), (override));
};

class MockMotor : public DcMotor {
public:
	MOCK_METHOD(bool, set, (float duty), (override));
	MOCK_METHOD(float, get, (), (const, override));
	MOCK_METHOD(void, enable, (), (override));
	MOCK_METHOD(void, disable, (), (override));
	MOCK_METHOD(bool, isOpenDirection, (), (const, override));
};

class MockVp3d : public ValueProvider3D {
public:
	MOCK_METHOD(float, getValue, (float xRpm, float y), (const, override));
};

class MockPwm : public SimplePwm {
public:
	MOCK_METHOD(void, setSimplePwmDutyCycle, (float dutyCycle), (override));
};

class MockOutputPin : public OutputPin {
public:
	MOCK_METHOD(void, setValue, (int value), (override));
};

class MockExecutor : public TestExecutor {
public:
	MOCK_METHOD(void, scheduleByTimestamp, (scheduling_s *scheduling, efitimeus_t timeUs, action_s action), (override));
	MOCK_METHOD(void, scheduleByTimestampNt, (scheduling_s *scheduling, efitime_t timeUs, action_s action), (override));
	MOCK_METHOD(void, scheduleForLater, (scheduling_s *scheduling, int delayUs, action_s action), (override));
};

class MockAirmass : public AirmassModelBase {
public:
	MockAirmass() : AirmassModelBase(veTable) {}

	MockVp3d veTable;

	MOCK_METHOD(AirmassResult, getAirmass, (int rpm), (override));
};
