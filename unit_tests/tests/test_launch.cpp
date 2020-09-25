#include "engine_test_helper.h"
#include "advance_map.h"
#include "engine_controller.h"
#include "launch_control.h"

extern WarningCodeState unitTestWarningCodeState;
extern LaunchControl Launch;

static const ignition_table_t mapBased16IgnitionTable = {
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
		{20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	20.000,	},
};


static void doRevolution(EngineTestHelper& eth, int periodMs) {
	float halfToothTime = (periodMs / 6.0f) / 2;

	eth.smartFireRise(halfToothTime);
	eth.fireFall(halfToothTime);
	eth.smartFireRise(halfToothTime);
	eth.fireFall(halfToothTime);
	eth.smartFireRise(halfToothTime);
	eth.fireFall(halfToothTime);

	// now missing tooth
	eth.smartFireRise(halfToothTime);
	eth.fireFall(3 * halfToothTime);

	// This tooth is the sync point!
	eth.smartFireRise(halfToothTime);
	eth.fireFall(halfToothTime);

    Launch.PeriodicTask();
}

TEST(launch_control, basicFunctionality) {
    
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
    setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth, IM_SEQUENTIAL);
	
    MEMCPY(config->ignitionTable, mapBased16IgnitionTable);

    // This is easiest to trip on a wheel that requires sync
	engineConfiguration->trigger.customTotalToothCount = 6;
	engineConfiguration->trigger.customSkippedToothCount = 1;
	eth.setTriggerType(TT_TOOTHED_WHEEL PASS_ENGINE_PARAMETER_SUFFIX);
	engineConfiguration->ambiguousOperationMode = FOUR_STROKE_CAM_SENSOR;

    // We do actually care about ign/inj 
    engineConfiguration->isInjectionEnabled = true;
    engineConfiguration->isIgnitionEnabled = true;

    //setupLaunchParams
    engineConfiguration->launchRpm = 4000;    // Rpm to trigger Launch condition
    engineConfiguration->launchTimingRetard = 10; // retard in absolute degrees ATDC
	engineConfiguration->launchTimingRpmRange = 500; // Rpm above Launch triggered for full retard
	engineConfiguration->launchSparkCutEnable = true;
	engineConfiguration->launchFuelCutEnable = true;
	engineConfiguration->hardCutRpmRange = 500; //Rpm above Launch triggered +(if retard enabled) launchTimingRpmRange to hard cut
	engineConfiguration->launchSpeedTreshold = 10; //maximum speed allowed before disable launch
	engineConfiguration->launchFuelAdded = 0; //UNUSED Extra fuel in % when launch are triggered
	engineConfiguration->launchBoostDuty = 70; //UNUSED boost valve duty cycle at launch
	engineConfiguration->launchActivateDelay = 3; //UNUSED Delay in seconds for launch to kick in
	engineConfiguration->enableLaunchRetard = true;
	engineConfiguration->enableLaunchBoost = true; //UNUSED
	engineConfiguration->launchSmoothRetard = true; //interpolates the advance linear from launchrpm to fully retarded at launchtimingrpmrange
	engineConfiguration->antiLagRpmTreshold = 3000;  //UNUSED 
    engineConfiguration->launchTpsTreshold = 50; //TPS value less then this, it will generate valid condition
    //setupActivationSpeed
    engineConfiguration->launchActivationMode = ALWAYS_ACTIVE_LAUNCH;
    //engineConfiguration->launchActivationMode = CLUTCH_INPUT_LAUNCH;
    //engineConfiguration->launchActivationMode = SWITCH_INPUT_LAUNCH;
    initTimingMap(PASS_ENGINE_PARAMETER_SIGNATURE);
    // Set predictable trigger settings
    //Sensor::setMockValue(SensorType::Tps1, 7);
	//eth.fireTriggerEvents2(/* count */ 100, 25 /* ms */);
    //setTargetRPM(1200,10);
 	doRevolution(eth, 42);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
	// now clear and advance more
	eth.clearQueue();    
 	doRevolution(eth, 41);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 40);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 39);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 38);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 37);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 36);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 35);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 34);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 33);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 32);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 31);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 30);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    std::cerr << "ignition angle" << getAdvance(GET_RPM(),50.0 PASS_ENGINE_PARAMETER_SUFFIX) << std::endl;
	// now clear and advance more
	eth.clearQueue();
 	doRevolution(eth, 29);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
    	// now clear and advance more
	eth.clearQueue();
	//ASSERT_EQ(1200,  GET_RPM()) << "RPM";
#ifdef all_Stuff
    //now move to start of launch
    eth.fireTriggerEvents2(/* count */ 100, 15 /* ms */);
    Sensor::setMockValue(SensorType::Tps1, 70);
    
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
	ASSERT_EQ(4000,  GET_RPM()) << "RPM";

	eth.fireTriggerEvents2(/* count */ 100, 12 /* ms */);
    std::cerr << "RPM: " << GET_RPM() << "\n" << std::endl;
	ASSERT_EQ(5000,  GET_RPM()) << "RPM";
#endif
} 