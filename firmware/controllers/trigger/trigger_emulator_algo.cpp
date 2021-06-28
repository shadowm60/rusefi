/**
 * @file trigger_emulator_algo.cpp
 *
 * This file is about producing real electrical signals which emulate trigger signal based on
 * a known TriggerWaveform.
 *
 * Historically this implementation was implemented based on PwmConfig which is maybe not the
 * best way to implement it. (todo: why is not the best way?)
 *
 * A newer implementation of pretty much the same thing is TriggerStimulatorHelper
 * todo: one emulator should be enough! another one should be eliminated
 *
 * @date Mar 3, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */
#include "state_sequence.h"
#include "global.h"
#include "efi_gpio.h"
#include "pin_repository.h"

int getPreviousIndex(const int currentIndex, const int size) {
	return (currentIndex + size - 1) % size;
}

bool needEvent(const int currentIndex, const int size, const MultiChannelStateSequence& mcss, int channelIndex) {
	int prevIndex = getPreviousIndex(currentIndex, size);
	pin_state_t previousValue = mcss.getChannelState(channelIndex, /*phaseIndex*/prevIndex);
	pin_state_t currentValue = mcss.getChannelState(channelIndex, /*phaseIndex*/currentIndex);

	return previousValue != currentValue;
}

#if EFI_EMULATE_POSITION_SENSORS

#include "engine.h"
#include "trigger_emulator_algo.h"
#include "engine_configuration.h"
#include "trigger_central.h"
#include "trigger_simulator.h"
#include "settings.h"
#include "pwm_generator_logic.h"

TriggerEmulatorHelper::TriggerEmulatorHelper() {
}

EXTERN_ENGINE;

static OutputPin emulatorOutputs[PWM_PHASE_MAX_WAVE_PER_PWM];

void TriggerEmulatorHelper::handleEmulatorCallback(const int size, const MultiChannelStateSequence& multiChannelStateSequence, int stateIndex DECLARE_ENGINE_PARAMETER_SUFFIX) {
	efitick_t stamp = getTimeNowNt();
	
	// todo: code duplication with TriggerStimulatorHelper::feedSimulatedEvent?

	for (size_t i = 0; i < PWM_PHASE_MAX_WAVE_PER_PWM; i++) {
		if (needEvent(stateIndex, size, multiChannelStateSequence, i)) {
			pin_state_t currentValue = multiChannelStateSequence.getChannelState(/*phaseIndex*/i, stateIndex);

			handleShaftSignal(i, currentValue, stamp PASS_ENGINE_PARAMETER_SUFFIX);
		}
	}
}

/*
 * todo: should we simply re-use instances used by trigger_decoder?
 * todo: since we are emulating same shape we are decoding
 */
static pin_state_t pinStates1[PWM_PHASE_MAX_COUNT];
static pin_state_t pinStates2[PWM_PHASE_MAX_COUNT];
static pin_state_t pinStates3[PWM_PHASE_MAX_COUNT];
static SingleChannelStateSequence waves[PWM_PHASE_MAX_WAVE_PER_PWM] = { SingleChannelStateSequence(pinStates1), SingleChannelStateSequence(pinStates2),
		SingleChannelStateSequence(pinStates3) };
static SingleChannelStateSequence sr[PWM_PHASE_MAX_WAVE_PER_PWM] = { waves[0], waves[1], waves[2] };

static float pwmSwitchTimesBuffer[PWM_PHASE_MAX_COUNT];

PwmConfig triggerSignal(pwmSwitchTimesBuffer, sr);

static int atTriggerVersion = 0;

#if EFI_ENGINE_SNIFFER
#include "engine_sniffer.h"
extern WaveChart waveChart;
#endif /* EFI_ENGINE_SNIFFER */

/**
 * todo: why is this method NOT reciprocal to getCrankDivider?!
 * todo: oh this method has only one usage? there must me another very similar method!
 */
static float getRpmMultiplier(operation_mode_e mode) {
	if (mode == FOUR_STROKE_SYMMETRICAL_CRANK_SENSOR) {
		return 2;
	} else if (mode == FOUR_STROKE_CAM_SENSOR) {
		return 0.5;
	} else if (mode == FOUR_STROKE_CRANK_SENSOR) {
		// unit test coverage still runs if the value below is changed to '2' not a great sign!
		return 1;
	}
	return 1;
}

void setTriggerEmulatorRPM(int rpm DECLARE_ENGINE_PARAMETER_SUFFIX) {
	engineConfiguration->triggerSimulatorFrequency = rpm;
	/**
	 * All we need to do here is to change the periodMs
	 * togglePwmState() would see that the periodMs has changed and act accordingly
	 */
	if (rpm == 0) {
		triggerSignal.setFrequency(NAN);
	} else {
		float rpmM = getRpmMultiplier(engine->getOperationMode(PASS_ENGINE_PARAMETER_SIGNATURE));
		float rPerSecond = rpm * rpmM / 60.0; // per minute converted to per second
		triggerSignal.setFrequency(rPerSecond);
	}
	engine->resetEngineSnifferIfInTestMode();

	efiPrintf("Emulating position sensor(s). RPM=%d", rpm);
}

static void updateTriggerWaveformIfNeeded(PwmConfig *state DECLARE_ENGINE_PARAMETER_SUFFIX) {
	if (atTriggerVersion < engine->triggerCentral.triggerShape.version) {
		atTriggerVersion = engine->triggerCentral.triggerShape.version;
		efiPrintf("Stimulator: updating trigger shape: %d/%d %d", atTriggerVersion,
				engine->getGlobalConfigurationVersion(), currentTimeMillis());


		TriggerWaveform *s = &engine->triggerCentral.triggerShape;
		pin_state_t *pinStates[PWM_PHASE_MAX_WAVE_PER_PWM] = {
				s->wave.channels[0].pinStates,
				s->wave.channels[1].pinStates,
				s->wave.channels[2].pinStates };
		copyPwmParameters(state, s->getSize(), s->wave.switchTimes, PWM_PHASE_MAX_WAVE_PER_PWM, pinStates);
		state->safe.periodNt = -1; // this would cause loop re-initialization
	}
}

static TriggerEmulatorHelper helper;
static bool hasStimPins = false;

static bool hasInitTriggerEmulator = false;

# if !EFI_UNIT_TEST

static void emulatorApplyPinState(int stateIndex, PwmConfig *state) /* pwm_gen_callback */ {
	if (engine->directSelfStimulation) {
		/**
		 * this callback would invoke the input signal handlers directly
		 */
		helper.handleEmulatorCallback(state->phaseCount,
				state->multiChannelStateSequence,
				stateIndex);
	}

#if EFI_PROD_CODE
	// Only set pins if they're configured - no need to waste the cycles otherwise
	else if (hasStimPins) {
		applyPinState(stateIndex, state);
	}
#endif /* EFI_PROD_CODE */
}

static void initTriggerPwm(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	// No need to start more than once
	if (hasInitTriggerEmulator) {
		return;
	}

	TriggerWaveform *s = &engine->triggerCentral.triggerShape;
	setTriggerEmulatorRPM(engineConfiguration->triggerSimulatorFrequency PASS_ENGINE_PARAMETER_SUFFIX);
	pin_state_t *pinStates[PWM_PHASE_MAX_WAVE_PER_PWM] = {
			s->wave.channels[0].pinStates,
			s->wave.channels[1].pinStates,
			s->wave.channels[2].pinStates };
	int phaseCount = s->getSize();
	assertIsInBounds(phaseCount - 1, pwmSwitchTimesBuffer, "pwmSwitchTimesBuffer");
	triggerSignal.weComplexInit("position sensor",
			&engine->executor,
			phaseCount, s->wave.switchTimes, PWM_PHASE_MAX_WAVE_PER_PWM,
			pinStates, updateTriggerWaveformIfNeeded, (pwm_gen_callback*)emulatorApplyPinState);

	hasInitTriggerEmulator = true;
}

void enableTriggerStimulator() {
	initTriggerPwm();
	engine->directSelfStimulation = true;
}

void enableExternalTriggerStimulator() {
	initTriggerPwm();
	engine->directSelfStimulation = false;
}

void disableTriggerStimulator() {
	engine->directSelfStimulation = false;
	triggerSignal.stop();
	hasInitTriggerEmulator = false;
}

void initTriggerEmulatorLogic(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	addConsoleActionI(CMD_RPM, setTriggerEmulatorRPM);
}

void onConfigurationChangeRpmEmulatorCallback(engine_configuration_s *previousConfiguration) {
	if (engineConfiguration->triggerSimulatorFrequency ==
			previousConfiguration->triggerSimulatorFrequency) {
		return;
	}
	setTriggerEmulatorRPM(engineConfiguration->triggerSimulatorFrequency);
}

void initTriggerEmulator(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	efiPrintf("Emulating %s", getConfigurationName(engineConfiguration->engineType));

	startTriggerEmulatorPins(PASS_ENGINE_PARAMETER_SIGNATURE);

	initTriggerEmulatorLogic(PASS_ENGINE_PARAMETER_SIGNATURE);
}

#endif /* EFI_UNIT_TEST */

void startTriggerEmulatorPins(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	hasStimPins = false;
	for (size_t i = 0; i < efi::size(emulatorOutputs); i++) {
		triggerSignal.outputPins[i] = &emulatorOutputs[i];

		brain_pin_e pin = CONFIG(triggerSimulatorPins)[i];

		// Only bother trying to set output pins if they're configured
		if (isBrainPinValid(pin)) {
			hasStimPins = true;
		}

#if EFI_PROD_CODE
		if (isConfigurationChanged(triggerSimulatorPins[i])) {
			triggerSignal.outputPins[i]->initPin("Trigger emulator", pin,
					&CONFIG(triggerSimulatorPinModes)[i]);
		}
#endif // EFI_PROD_CODE
	}
}

void stopTriggerEmulatorPins() {
#if EFI_PROD_CODE
	for (size_t i = 0; i < efi::size(emulatorOutputs); i++) {
		if (isConfigurationChanged(triggerSimulatorPins[i])) {
			triggerSignal.outputPins[i]->deInit();
		}
	}
#endif // EFI_PROD_CODE
}

#endif /* EFI_EMULATE_POSITION_SENSORS */
