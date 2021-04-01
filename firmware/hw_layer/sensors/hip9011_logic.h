/*
 * @file hip9011_logic.h
 *
 *  Created on: Jan 3, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "efifeatures.h"
#include "rusefi_enums.h"

#define PIF						3.14159f
#define HIP9011_BAND(bore) (900 / (PIF * (bore) / 2))

#define INT_LOOKUP_SIZE 		32
#define GAIN_LOOKUP_SIZE 		64
#define BAND_LOOKUP_SIZE 		64

/**
 * this interface defines hardware communication layer for HIP9011 chip
 */
class Hip9011HardwareInterface {
public:
	virtual int sendSyncCommand(unsigned char command, uint8_t *rx_ptr) = 0;
};

#if EFI_PROD_CODE || EFI_SIMULATOR
#define PASS_HIP_PARAMS
#define DEFINE_HIP_PARAMS
#define GET_CONFIG_VALUE(x) CONFIG(x)
#define FORWARD_HIP_PARAMS
#define DEFINE_PARAM_SUFFIX(x)
#else

#define PASS_HIP_PARAMS CONFIG(knockBandCustom), \
		CONFIG(cylinderBore), \
		CONFIG(hip9011Gain), \
		CONFIG(hip9011PrescalerAndSDO), \
		CONFIG(knockDetectionWindowStart), \
		CONFIG(knockDetectionWindowEnd)

#define FORWARD_HIP_PARAMS knockBandCustom, \
		cylinderBore, \
		hip9011Gain, \
		hip9011PrescalerAndSDO, \
		knockDetectionWindowStart, \
		knockDetectionWindowEnd

#define DEFINE_HIP_PARAMS float knockBandCustom,\
		float cylinderBore, \
		float hip9011Gain, \
		int hip9011PrescalerAndSDO, \
		float knockDetectionWindowStart, \
		float knockDetectionWindowEnd


#define GET_CONFIG_VALUE(x) x
#define DEFINE_PARAM_SUFFIX(x) , x
#endif

class HIP9011 {
public:
	explicit HIP9011(Hip9011HardwareInterface *hardware);
	int sendCommand(uint8_t cmd);

	float getRpmByAngleWindowAndTimeUs(int timeUs, float angleWindowWidth);
	void prepareRpmLookup(void);
	void setAngleWindowWidth(DEFINE_HIP_PARAMS);
	void handleSettings(int rpm DEFINE_PARAM_SUFFIX(DEFINE_HIP_PARAMS));
	float getBand(DEFINE_HIP_PARAMS);
	int getIntegrationIndexByRpm(float rpm);
	int getBandIndex(DEFINE_HIP_PARAMS);
	int getGainIndex(DEFINE_HIP_PARAMS);

	/* Settings loaded to chip */
	uint8_t intergratorIdx = 0xff;
	uint8_t bandIdx = 0xff;
	uint8_t prescaler = 0xff;
	uint8_t gainIdx = 0xff;
	uint8_t channelIdx = 0xff;

	int correctResponsesCount = 0;
	int invalidResponsesCount = 0;
	float angleWindowWidth = - 1;

	int totalKnockEventsCount = 0;
	Hip9011HardwareInterface *hw;
	bool adv_mode = false;
	/**
	 * Int/Hold pin is controlled from scheduler call-backs which are set according to current RPM
	 *
	 * The following state makes sure that we only have SPI communication while not integrating and that we take
	 * a good ADC reading after integrating.
	 *
	 * Once integration window is over, we wait for the 2nd ADC callback and then initiate SPI communication if needed
	 *
	 * hipOutput should be set to used FAST adc device
	 */
	hip_state_e state;
	uint8_t cylinderNumber;
	int raw_value;

	/* error counters */
	int overrun = 0;

	float rpmLookup[INT_LOOKUP_SIZE];
};

// 0b010x.xxxx
#define SET_PRESCALER_CMD(v) 	(0x40 | ((v) & 0x1f))
// 0b1110.000x
#define SET_CHANNEL_CMD(v) 		(0xE0 | ((v) & 0x01))
// 0b00xx.xxxx
#define SET_BAND_PASS_CMD(v)	(0x00 | ((v) & 0x3f))
// 0b10xx.xxxx
#define SET_GAIN_CMD(v)			(0x80 | ((v) & 0x3f))
// 0b110x.xxxx
#define SET_INTEGRATOR_CMD(v)	(0xC0 | ((v) & 0x1f))
// 0b0111.0001
#define SET_ADVANCED_MODE_CMD	(0x71)

//	D[4:1] = 0000 : 4 MHz
#define HIP_4MHZ_PRESCALER		(0x0 << 1)
//	D[4:1] = 0001 : 5 MHz
#define HIP_5MHZ_PRESCALER		(0x1 << 1)
//	D[4:1] = 0010 : 6 MHz
#define HIP_6MHZ_PRESCALER		(0x2 << 1)
//	D[4:1] = 0011 ; 8 MHz
#define HIP_8MHZ_PRESCALER		(0x3 << 1)
//	D[4:1] = 0100 ; 10 MHz
#define HIP_10MHZ_PRESCALER		(0x4 << 1)
//	D[4:1] = 0101 ; 12 MHz
#define HIP_12MHZ_PRESCALER		(0x5 << 1)
//	D[4:1] = 0110 : 16 MHz
#define HIP_16MHZ_PRESCALER		(0x6 << 1)
//	D[4:1] = 0111 : 20 MHz
#define HIP_20MHZ_PRESCALER		(0x7 << 1)
//	D[4:1] = 1000 : 24 MHz
#define HIP_24MHZ_PRESCALER		(0x8 << 1)

