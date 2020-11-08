/*
 * @file tooth_logger.cpp
 *
 * @date Jul 7, 2019
 * @author Matthew Kennedy
 */

#include "tooth_logger.h"

#include "global.h"
#include "perf_trace.h"

#if EFI_TOOTH_LOGGER

EXTERN_ENGINE;

#include <cstddef>
#include "efitime.h"
#include "efilib.h"
#include "tunerstudio_outputs.h"

typedef struct __attribute__ ((packed)) {
	// the whole order of all packet bytes is reversed, not just the 'endian-swap' integers
	uint32_t timestamp;
	// unfortunately all these fields are required by TS...
	bool priLevel : 1;
	bool secLevel : 1;
	bool trigger : 1;
	bool sync : 1;
	bool coil : 1;
	bool injector : 1;
	//use to signal where the error occured
	bool trigErr : 1;
} composite_logger_s;

/**
 * Engine idles around 20Hz and revs up to 140Hz, at 60/2 and 8 cylinders we have about 20Khz events
 * If we can read buffer at 50Hz we want buffer to be about 400 elements.
 */
static composite_logger_s buffer[COMPOSITE_PACKET_COUNT] CCM_OPTIONAL;
static composite_logger_s *ptr_buffer_first = &buffer[0];
static composite_logger_s *ptr_buffer_second = &buffer[(COMPOSITE_PACKET_COUNT/2)-1];

/** 
 * For the SD_Card logger we use a new buffer, since sdcard transfer is fast we do not need all these 
 * splitting and processing, and even the buffer could be smaller since we need a bit of history and 
 * a few points after the trigger.
 * Note: considering 60/2 with trigger at every tooth -> 116 events
 *       cosnidering cam shaft signals consider 10 tooth max? -> 20 events
 *       consider 1 extra event for error storage
 * 	=> total of 137 so lets make it 150 events.
 * 
 * Note: this will be a circular buffer always running.
 */
static  composite_logger_s sd_buffer[SD_CARD_BUFFER_SIZE] CCM_OPTIONAL;

static size_t idx = 0;
//where in the buffer to we want to have the trigger, let's place it in the middle
static size_t triggerPosition = 0; 
/* better count then calculate at every index if we are there, saves cpu time */
static size_t triggerCnt = 0;
static bool saveEvents = true;
static uint32_t markCount = 0;


static size_t NextIdx = 0;
static volatile bool ToothLoggerEnabled = false;
static volatile bool firstBuffer = true;
static uint32_t lastEdgeTimestamp = 0;

static bool trigger1 = false; //current cranck value
static bool trigger2 = false; //current CAM value
static bool trigger = false;  //always true if we have a change in CAM (TS dependent)

// any coil, all coils thrown together
static bool coil = false;
// same about injectors
static bool injector = false;
// trigger error 
static bool triggerError = false;

int getCompositeRecordCount() {
	return NextIdx;
}

#if EFI_UNIT_TEST
#include "logicdata.h"
int copyCompositeEvents(CompositeEvent *events) {
	for (int i = 0;i < NextIdx;i++) {
		CompositeEvent *event = &events[i];
		event->timestamp = SWAP_UINT32(buffer[i].timestamp);
		event->primaryTrigger = buffer[i].priLevel;
		event->secondaryTrigger = buffer[i].secLevel;
		event->isTDC = buffer[i].trigger;
		event->sync = buffer[i].sync;
		event->coil = buffer[i].coil;
		event->injector = buffer[i].injector;
	}
	return NextIdx;
}

#endif // EFI_UNIT_TEST

static void SetNextEntryToRAM(efitick_t timestamp, bool trigger1, bool trigger2,
		bool trigger DECLARE_ENGINE_PARAMETER_SUFFIX) {
	static uint32_t lastTimeStamp = 0;
	uint32_t nowUs = NT2US(timestamp);
	uint32_t deltaTimeStamp = 0;

	if (saveEvents) {
		// TS uses big endian, grumble
		sd_buffer[idx].timestamp = (nowUs/10); //SWAP_UINT32(nowUs);
		sd_buffer[idx].priLevel = trigger1;
		sd_buffer[idx].secLevel = trigger2;
		sd_buffer[idx].trigger = trigger;
		sd_buffer[idx].sync = engine->triggerCentral.triggerState.shaft_is_synchronized;
		sd_buffer[idx].coil = coil;
		sd_buffer[idx].injector = injector;
		sd_buffer[idx].trigErr = triggerError;

		idx++;

		if (idx >= (SD_CARD_BUFFER_SIZE - 1))
		{
			//wrap around
			idx = 0;
		}

		/* EVENTS_AFTER_ERROR */
		if (triggerError)
		{
			triggerCnt++;

			if (triggerCnt >= EVENTS_AFTER_ERROR) {
				/* time to save it to sd_card */
				saveEvents = false;
			}
		}
	} else {
		/* we cannot save events right now */
	}

}

static void SetNextCompositeEntry(efitick_t timestamp, bool trigger1, bool trigger2,
		bool trigger DECLARE_ENGINE_PARAMETER_SUFFIX) {
	uint32_t nowUs = NT2US(timestamp);
	
	// TS uses big endian, grumble
	buffer[NextIdx].timestamp = SWAP_UINT32(nowUs);
	buffer[NextIdx].priLevel = trigger1;
	buffer[NextIdx].secLevel = trigger2;
	buffer[NextIdx].trigger = trigger;
	buffer[NextIdx].sync = engine->triggerCentral.triggerState.shaft_is_synchronized;
	buffer[NextIdx].coil = coil;
	buffer[NextIdx].injector = injector;
	buffer[NextIdx].trigErr = triggerError;

	NextIdx++;

	static_assert(sizeof(composite_logger_s) == COMPOSITE_PACKET_SIZE, "composite packet size");

	//If we hit the end, loop
	if ((firstBuffer) && (NextIdx >= (COMPOSITE_PACKET_COUNT/2))) {
		/* first half is full */
#if EFI_TUNER_STUDIO		
		tsOutputChannels.toothLogReady = true;
#endif		
		firstBuffer = false;
	}
	if ((!firstBuffer) && (NextIdx >= sizeof(buffer) / sizeof(buffer[0]))) {
#if EFI_TUNER_STUDIO		
		tsOutputChannels.toothLogReady = true;
#endif		
		NextIdx = 0;
		firstBuffer = true;
	}
}

void LogTriggerTooth(trigger_event_e tooth, efitick_t timestamp DECLARE_ENGINE_PARAMETER_SUFFIX) {

	//we do save the change locally even if we are not preparing the data to TS
	switch (tooth) {
	case SHAFT_PRIMARY_FALLING:
		trigger1 = false;
		trigger = false;
		break;
	case SHAFT_PRIMARY_RISING:
		trigger1 = true;
		trigger = false;
		break;
	case SHAFT_SECONDARY_FALLING:
		trigger2 = false;
		trigger = true;
		break;
	case SHAFT_SECONDARY_RISING:
		trigger2 = true;
		trigger = true;
		break;
	default:
		break;
	}

	SetNextEntryToRAM(timestamp, trigger1, trigger2, trigger PASS_ENGINE_PARAMETER_SUFFIX);

	// bail if we aren't enabled
	if (!ToothLoggerEnabled) {
		return;
	}

	// Don't log at significant engine speed
	if (engine->rpmCalculator.getRpm() > 4000) {
		return;
	}

	ScopePerf perf(PE::LogTriggerTooth);

/*
		// We currently only support the primary trigger falling edge
    	// (this is the edge that VR sensors are accurate on)
    	// Since VR sensors are the most useful case here, this is okay for now.
    	if (tooth != SHAFT_PRIMARY_FALLING) {
    		return;
    	}

    	uint32_t nowUs = NT2US(timestamp);
    	// 10us per LSB - this gives plenty of accuracy, yet fits 655.35 ms in to a uint16
    	uint16_t delta = static_cast<uint16_t>((nowUs - lastEdgeTimestamp) / 10);
    	lastEdgeTimestamp = nowUs;

    	SetNextEntry(delta);
*/
	SetNextCompositeEntry(timestamp, trigger1, trigger2, trigger PASS_ENGINE_PARAMETER_SUFFIX);
}

void LogTriggerTopDeadCenter(efitick_t timestamp DECLARE_ENGINE_PARAMETER_SUFFIX) {
	// bail if we aren't enabled
	if (!ToothLoggerEnabled) {
		return;
	}
	UNUSED(timestamp);
	//SetNextCompositeEntry(timestamp, trigger1, trigger2, true PASS_ENGINE_PARAMETER_SUFFIX);
	//SetNextCompositeEntry(timestamp + 10, trigger1, trigger2, false PASS_ENGINE_PARAMETER_SUFFIX);
}

void LogTriggerCoilState(efitick_t timestamp, bool state DECLARE_ENGINE_PARAMETER_SUFFIX) {
	if (!ToothLoggerEnabled) {
		return;
	}
	coil = state;
	UNUSED(timestamp);
	//SetNextCompositeEntry(timestamp, trigger1, trigger2, trigger PASS_ENGINE_PARAMETER_SUFFIX);
}

void LogTriggerInjectorState(efitick_t timestamp, bool state DECLARE_ENGINE_PARAMETER_SUFFIX) {
	if (!ToothLoggerEnabled) {
		return;
	}
	injector = state;
	UNUSED(timestamp);
	//SetNextCompositeEntry(timestamp, trigger1, trigger2, trigger PASS_ENGINE_PARAMETER_SUFFIX);
}

void EnableToothLogger() {
	// Clear the buffer
	memset(buffer, 0, sizeof(buffer));

	// Reset the last edge to now - this prevents the first edge logged from being bogus
	lastEdgeTimestamp = getTimeNowUs();

	// Reset write index
	NextIdx = 0;

	// Enable logging of edges as they come
	ToothLoggerEnabled = true;

#if EFI_TUNER_STUDIO
	// Tell TS that we're ready for it to read out the log
	// nb: this is a lie, as we may not have written anything
	// yet.  However, we can let it continuously read out the buffer
	// as we update it, which looks pretty nice.
	tsOutputChannels.toothLogReady = false;
#endif // EFI_TUNER_STUDIO
}

void EnableToothLoggerIfNotEnabled() {
	if (!ToothLoggerEnabled) {
		EnableToothLogger();
	}
}

void DisableToothLogger() {
	ToothLoggerEnabled = false;
#if EFI_TUNER_STUDIO
	tsOutputChannels.toothLogReady = false;
#endif // EFI_TUNER_STUDIO
}

ToothLoggerBuffer GetToothLoggerBuffer() {
	if (firstBuffer) {
#if EFI_TUNER_STUDIO		
		tsOutputChannels.toothLogReady = false;
#endif		
		return { reinterpret_cast<uint8_t*>(ptr_buffer_second), (sizeof(buffer)/2) };
	} else {
#if EFI_TUNER_STUDIO		
		tsOutputChannels.toothLogReady = false;
#endif		
		return { reinterpret_cast<uint8_t*>(ptr_buffer_first), (sizeof(buffer)/2) };
	}
}


void LogTriggerError(obd_code_e code DECLARE_ENGINE_PARAMETER_SUFFIX)
{
	if ((code == CUSTOM_OBD_TRIGGER_WAVEFORM) ||
	    (code ==  CUSTOM_SYNC_COUNT_MISMATCH) ||
		(code == CUSTOM_ERR_VVT_OUT_OF_RANGE)) {
			efitick_t nowNt = getTimeNowNt();
			if (triggerError == false) {
				triggerError = true;

				/* ensure that we log the moment when it all happens */
				SetNextEntryToRAM(nowNt, trigger1, trigger2, trigger PASS_ENGINE_PARAMETER_SUFFIX);

				/* index where it all happened */
				triggerPosition = idx;
			} else {
				/* we cannot save a new error till we are over dumping the current on. */
				/* we might need to log it somehow, but for now we only log the first */
			}
		} else {
			/* not yet supported */
			
		}
}

size_t LogTriggerEventToSd(char *ptr) {
	static size_t eventCount = 0;
	static uint32_t deltaTime;
	static uint32_t prevTime;
	if (ptr != NULL) {
		if (saveEvents == false) {
			/* we have ram contant we need to shift out 				 */
			/* let's try to do it in multiple steps to avoid big buffers */
			/* we always start from where we stopped logging             */
			/* shift out line by line */

			/* example is:												 */
			/* Flag,Flag,Flag,Flag,Flag,ms,ms                            */
			/* 1.0,1.0,0.0,1.0,0.0,2398720.2,1805.49                     */
			if (eventCount == 0) {
				deltaTime = sd_buffer[idx].timestamp;
				prevTime = deltaTime;
			} else {
				deltaTime = sd_buffer[idx].timestamp - prevTime;
				prevTime = sd_buffer[idx].timestamp;
			}
			sprintf(ptr,"%d.0,%d.0,%d.0,%d.0,%d.0,%ld.%ld,%ld.%ld\n",\
			sd_buffer[idx].priLevel, \
			sd_buffer[idx].secLevel, \
			sd_buffer[idx].trigger, \
			sd_buffer[idx].sync, \
			sd_buffer[idx].trigErr, \
			(sd_buffer[idx].timestamp/100), ((sd_buffer[idx].timestamp%100)/10),\
			(deltaTime/100),(deltaTime%100));

			eventCount++;
			idx++;

			if (idx >= (SD_CARD_BUFFER_SIZE - 1)) {
				//wrap around
				idx = 0;
			}	

			if (eventCount >= (SD_CARD_BUFFER_SIZE - 1)){
				eventCount = 0;
				triggerPosition = 0;
				triggerCnt = 0;
				triggerError = 0;
				saveEvents = true;

				/* we need to add MARK idx */
				uint32_t len = efiStrlen(ptr);
				sprintf(ptr+len,"MARK %03ld\n",markCount);
				markCount++;

			}

			return efiStrlen(ptr);
		} else {
			/* we do not have data to log into sd */
			return 0;
		}
	} else {
		/* no buffer available, so no data is saved */
		return 0;
	}
}

#endif /* EFI_TOOTH_LOGGER */
