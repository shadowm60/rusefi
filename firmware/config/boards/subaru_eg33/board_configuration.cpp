/**
 * @file boards/subaru_eg33/board_configuration.h
 *
 * @brief In this file we can override engine_configuration.cpp.
 *
 * @date Feb 06, 2021
 * @author Andrey Gusakov, 2021
 */

#include "global.h"
#include "engine.h"
#include "engine_math.h"
#include "allsensors.h"
#include "fsio_impl.h"
#include "engine_configuration.h"
#include "smart_gpio.h"

EXTERN_ENGINE;

void setPinConfigurationOverrides(void) {

}

void setSerialConfigurationOverrides(void) {
	engineConfiguration->useSerialPort = false;
	engineConfiguration->binarySerialTxPin = GPIOE_1;
	engineConfiguration->binarySerialRxPin = GPIOE_0;
	/* actually Bluetooth/WiFi interface */
	//engineConfiguration->consoleSerialTxPin = GPIOC_10;
	//engineConfiguration->consoleSerialRxPin = GPIOC_11;
	engineConfiguration->tunerStudioSerialSpeed = SERIAL_SPEED;
	engineConfiguration->uartConsoleSerialSpeed = SERIAL_SPEED;
}

void setSdCardConfigurationOverrides(void) {
	engineConfiguration->is_enabled_spi_1 = false;
	engineConfiguration->sdCardSpiDevice = SPI_DEVICE_1;
	engineConfiguration->sdCardCsPin = GPIOA_2;
	engineConfiguration->isSdCardEnabled = false;
}

/**
 * @brief   Board-specific configuration code overrides.
 * @todo    Add your board-specific code, if any.
 */
void setBoardConfigurationOverrides(void) {
	setSerialConfigurationOverrides();

	/* Battery voltage */
	engineConfiguration->vbattAdcChannel = EFI_ADC_6;
	/* Throttle position */
	engineConfiguration->tps1_1AdcChannel = EFI_ADC_12;
	/* MAP */
	engineConfiguration->map.sensor.hwChannel = EFI_ADC_10;
	/* MAF */
	engineConfiguration->mafAdcChannel = EFI_ADC_3;
	/* coolant t */
	engineConfiguration->clt.adcChannel = EFI_ADC_14;
	/* not yet */
	engineConfiguration->iat.adcChannel = EFI_ADC_NONE;
	/* narrow */
	engineConfiguration->afr.hwChannel = EFI_ADC_NONE;

	engineConfiguration->adcVcc = ADC_VCC;

	engineConfiguration->baroSensor.hwChannel = EFI_ADC_NONE;
	engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_NONE;

	/* Injectors */
	engineConfiguration->injectionPins[1 - 1] = MC33810_0_OUT_0;
	engineConfiguration->injectionPins[2 - 1] = MC33810_1_OUT_0;
	engineConfiguration->injectionPins[3 - 1] = MC33810_0_OUT_1;
	engineConfiguration->injectionPins[4 - 1] = MC33810_1_OUT_1;
	engineConfiguration->injectionPins[5 - 1] = MC33810_0_OUT_2;
	engineConfiguration->injectionPins[6 - 1] = MC33810_1_OUT_2;
	engineConfiguration->injectionPins[7 - 1] = MC33810_0_OUT_3;
	engineConfiguration->injectionPins[8 - 1] = MC33810_1_OUT_3;

	/* Ignition */
	engineConfiguration->ignitionPins[1 - 1] = MC33810_0_GD_0;
	engineConfiguration->ignitionPins[2 - 1] = MC33810_1_GD_1;
	engineConfiguration->ignitionPins[3 - 1] = MC33810_0_GD_1;
	engineConfiguration->ignitionPins[4 - 1] = MC33810_1_GD_0;
	engineConfiguration->ignitionPins[5 - 1] = MC33810_0_GD_3;
	engineConfiguration->ignitionPins[6 - 1] = MC33810_1_GD_2;
	engineConfiguration->ignitionPins[7 - 1] = MC33810_0_GD_2;
	engineConfiguration->ignitionPins[8 - 1] = MC33810_1_GD_3;
	//engineConfiguration->ignitionPinMode = OM_INVERTED;

	// Vbat divider: 10K + 1K
	engineConfiguration->vbattDividerCoeff = (1.0 + 10.0) / 1.0;
	//engineConfiguration->clt.config.bias_resistor = 2700;
	//sengineConfiguration->iat.config.bias_resistor = 2700;

	// Idle configuration
	engineConfiguration->useStepperIdle = false;
	engineConfiguration->isDoubleSolenoidIdle = true;
	engineConfiguration->idle.solenoidPin = TLE6240_PIN_11;
	engineConfiguration->secondSolenoidPin = TLE6240_PIN_12;

	engineConfiguration->communicationLedPin = GPIOG_6;	/* LD1 - green */
	engineConfiguration->runningLedPin = GPIOG_8; /* LD3 - yellow */
	engineConfiguration->warningLedPin = GPIO_UNASSIGNED; 	/* LD3 - yellow*/
	//engineConfiguration->unusedErrorPin = LED_ERROR_BRAIN_PIN;	/* LD2 - red */

	/* IF you have BOTH camshaft position sensor and crankshaft position sensor
	 * camshaft is always trigger#1 input and then crankshaft is trigger#2. */
	engineConfiguration->triggerInputPins[0] = GPIOH_12;	/* cam */
	engineConfiguration->triggerInputPins[1] = GPIOH_10;	/* crank pos #1 */
	engineConfiguration->triggerInputPins[2] = GPIOE_9;		/* crank pos #2 */
	engineConfiguration->camInputs[0] = GPIO_UNASSIGNED;

	/* SPI devices: mess of board and engine configurations */
	/* TLE6240 */
	engineConfiguration->tle6240spiDevice = SPI_DEVICE_4;
	engineConfiguration->tle6240_cs = GPIOE_15;	/* SPI4_NSS0 */
	engineConfiguration->tle6240_csPinMode = OM_OPENDRAIN;
	/* MC33972 */
	engineConfiguration->mc33972spiDevice = SPI_DEVICE_4;
	engineConfiguration->mc33972_cs = GPIOE_10;	/* SPI4_NSS2 */
	engineConfiguration->mc33972_csPinMode = OM_OPENDRAIN;

	/* TLE6240 - OUT3, also PG2 */
	engineConfiguration->tachOutputPin = TLE6240_PIN_2;
	engineConfiguration->tachOutputPinMode = OM_DEFAULT;
	/* spi driven - TLE6240 - OUT5 */
#if 0
	engineConfiguration->fuelPumpPin = TLE6240_PIN_5;
	engineConfiguration->fuelPumpPinMode = OM_DEFAULT;
	/* self shutdown? */
	engineConfiguration->mainRelayPin = GPIOH_7;
	engineConfiguration->mainRelayPinMode = OM_DEFAULT;
#else
	engineConfiguration->fuelPumpPin = GPIO_UNASSIGNED;
	engineConfiguration->fuelPumpPinMode = OM_DEFAULT;
	/* self shutdown? */
	engineConfiguration->mainRelayPin = TLE6240_PIN_5;
	engineConfiguration->mainRelayPinMode = OM_DEFAULT;
#endif
	/* spi driven - TLE6240 - OUT1, OUT2 */
	engineConfiguration->fanPin = TLE6240_PIN_1;
	engineConfiguration->fanPinMode = OM_DEFAULT;
	/* TODO: second fan */
	//engineConfiguration->fanPin[1] = TLE6240_PIN_2;
	//engineConfiguration->fanPinMode[1] = OM_DEFAULT;
	/* spi driven - TLE6240 - OUT8 */
	engineConfiguration->malfunctionIndicatorPin = TLE6240_PIN_7;
	engineConfiguration->malfunctionIndicatorPinMode = OM_DEFAULT;

	// starter block
	/* Starter signal connected through MC33972 - SG11 */
	//setFsio(0, (GPIOB_1), STARTER_RELAY_LOGIC PASS_CONFIG_PARAMETER_SUFFIX);

	// not used
	engineConfiguration->externalKnockSenseAdc = EFI_ADC_NONE;
	engineConfiguration->displayMode = DM_NONE;
	engineConfiguration->HD44780_rs = GPIO_UNASSIGNED;
	engineConfiguration->HD44780_e = GPIO_UNASSIGNED;
	engineConfiguration->HD44780_db4 = GPIO_UNASSIGNED;
	engineConfiguration->HD44780_db5 = GPIO_UNASSIGNED;
	engineConfiguration->HD44780_db6 = GPIO_UNASSIGNED;
	engineConfiguration->HD44780_db7 = GPIO_UNASSIGNED;
	engineConfiguration->digitalPotentiometerChipSelect[0] = GPIO_UNASSIGNED;
	engineConfiguration->digitalPotentiometerChipSelect[1] = GPIO_UNASSIGNED;
	engineConfiguration->digitalPotentiometerChipSelect[2] = GPIO_UNASSIGNED;
	engineConfiguration->digitalPotentiometerChipSelect[3] = GPIO_UNASSIGNED;
	engineConfiguration->vehicleSpeedSensorInputPin = GPIO_UNASSIGNED;

	engineConfiguration->digitalPotentiometerSpiDevice = SPI_NONE;
	engineConfiguration->max31855spiDevice = SPI_NONE;

	/////////////////////////////////////////////////////////

	engineConfiguration->is_enabled_spi_1 = true;
	engineConfiguration->is_enabled_spi_2 = false;
	engineConfiguration->is_enabled_spi_3 = true;
	engineConfiguration->is_enabled_spi_4 = true;

	engineConfiguration->spi1mosiPin = GPIO_UNASSIGNED;
	engineConfiguration->spi1MosiMode = PO_DEFAULT;
	engineConfiguration->spi1misoPin = GPIO_UNASSIGNED;
	engineConfiguration->spi1MisoMode = PO_DEFAULT;
	engineConfiguration->spi1sckPin = GPIO_UNASSIGNED;
	engineConfiguration->spi1SckMode = PO_DEFAULT;

	engineConfiguration->spi3mosiPin = GPIOC_12;
	engineConfiguration->spi3MosiMode = PO_DEFAULT;
	engineConfiguration->spi3misoPin = GPIOC_11;
	engineConfiguration->spi3MisoMode = PO_DEFAULT;
	engineConfiguration->spi3sckPin = GPIOC_10;
	engineConfiguration->spi3SckMode = PO_DEFAULT;

	/* Knock sensor */
	engineConfiguration->hip9011SpiDevice = SPI_DEVICE_4;
	engineConfiguration->hip9011CsPin = GPIOE_11;	/* SPI4_NSS1 */
	engineConfiguration->hip9011CsPinMode = OM_OPENDRAIN;
	engineConfiguration->hip9011IntHoldPin = GPIOH_8;
	engineConfiguration->hip9011IntHoldPinMode = OM_OPENDRAIN;
	engineConfiguration->hipOutputChannel = EFI_ADC_7; /* PA7 */
	engineConfiguration->isHip9011Enabled = true;


	engineConfiguration->hip9011PrescalerAndSDO = (0x6 << 1); //HIP_16MHZ_PRESCALER;
	engineConfiguration->hip9011Gain = 1.0;
	engineConfiguration->knockBandCustom = 0.0;
	engineConfiguration->knockVThreshold = 4.0;
	engineConfiguration->cylinderBore = 96.9;
	engineConfiguration->maxKnockSubDeg = 20.0;

#if 0
	engineConfiguration->cj125SpiDevice = SPI_DEVICE_3;
	engineConfiguration->cj125ua = EFI_ADC_9;
	engineConfiguration->cj125ur = EFI_ADC_12;
	engineConfiguration->cj125CsPin = GPIOA_15;
	engineConfiguration->cj125CsPinMode = OM_OPENDRAIN;
	engineConfiguration->wboHeaterPin = GPIOC_13;
	engineConfiguration->o2heaterPin = GPIOC_13;
#endif
	engineConfiguration->isCJ125Enabled = false;

	engineConfiguration->canTxPin = GPIOD_1;
	engineConfiguration->canRxPin = GPIOD_0;

	/* not used pins with testpads */
	engineConfiguration->triggerSimulatorPins[0] = GPIOH_2;
	engineConfiguration->triggerSimulatorPins[1] = GPIOH_3;
	engineConfiguration->triggerSimulatorPins[2] = GPIOH_4;
	engineConfiguration->triggerSimulatorPinModes[0] = OM_DEFAULT;
	engineConfiguration->triggerSimulatorPinModes[1] = OM_DEFAULT;
	engineConfiguration->triggerSimulatorPinModes[2] = OM_DEFAULT;

	engineConfiguration->logicAnalyzerPins[0] = GPIO_UNASSIGNED;
	engineConfiguration->logicAnalyzerPins[1] = GPIO_UNASSIGNED;
	engineConfiguration->logicAnalyzerPins[2] = GPIO_UNASSIGNED;
	engineConfiguration->logicAnalyzerPins[3] = GPIO_UNASSIGNED;
	//!!!!!!!!!!!!!!!!!!!
	//engineConfiguration->silentTriggerError = true;

	//!!!!!!!!!!!!!
	//engineConfiguration->isEngineChartEnabled = false;

	if (engineConfiguration->fuelAlgorithm == LM_REAL_MAF)
		setAlgorithm(LM_SPEED_DENSITY PASS_CONFIG_PARAMETER_SUFFIX);
	if (engineConfiguration->fuelAlgorithm == LM_ALPHA_N)
		setAlgorithm(LM_ALPHA_N PASS_CONFIG_PARAMETER_SUFFIX);
}
