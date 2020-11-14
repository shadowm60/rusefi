/*
 * @file smart_gpio.cpp
 *
 * @date Apr 13, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "global.h"

#if EFI_PROD_CODE
#include "smart_gpio.h"
#include "efi_gpio.h"
#include "engine_configuration.h"
#include "hardware.h"
#include "mpu_util.h"
#include "gpio_ext.h"
#include "pin_repository.h"
#include "drivers/gpio/tle6240.h"
#include "drivers/gpio/mc33972.h"
#include "drivers/gpio/mc33810.h"
#include "drivers/gpio/tle8888.h"
#include "drivers/gpio/drv8860.h"

EXTERN_CONFIG;
static OutputPin tle8888Cs;
static OutputPin tle6240Cs;
static OutputPin mc33972Cs;
static OutputPin drv8860Cs;

// todo: migrate to TS or board config
#ifndef TLE6240_RESET_PORT
#define TLE6240_RESET_PORT GPIOG
#endif /* TLE6240_RESET_PORT */
#ifndef TLE6240_RESET_PAD
#define TLE6240_RESET_PAD 3U
#endif /* TLE6240_RESET_PAD */
#ifndef TLE6240_DIRECT_IO
#define TLE6240_DIRECT_IO \
		/* IN1  - D_TACH_OUT */ \
		[0] = {.port = GPIOG, .pad = 2}, \
		/* IN2..4 grounded */ \
		[1] = {.port = NULL, .pad = 0}, \
		[2] = {.port = NULL, .pad = 0}, \
		[3] = {.port = NULL, .pad = 0}, \
		/* IN9  - D_INJ_5 */ \
		[4] = {.port = GPIOD, .pad = 15}, \
		/* IN10 - D_WASTGATE */ \
		[5] = {.port = GPIOD, .pad = 14}, \
		/* IN11 - D_IDLE_OPEN */ \
		[6] = {.port = GPIOC, .pad = 6}, \
		/* IN12 - D_IDLE_CLOSE */ \
		[7] = {.port = GPIOC, .pad = 7},
#endif /* TLE6240_DIRECT_IO */

#if (BOARD_TLE6240_COUNT > 0)
struct tle6240_config tle6240 = {
	.spi_bus = NULL /* TODO software lookup &SPID4 */,
	.spi_config = {
		.circular = false,
		.end_cb = NULL,
		.ssport = NULL,
		.sspad = 0,
		.cr1 =
			SPI_CR1_16BIT_MODE |
			SPI_CR1_SSM |
			SPI_CR1_SSI |
			/* SPI_CR1_LSBFIRST | */
			((3 << SPI_CR1_BR_Pos) & SPI_CR1_BR) |	/* div = 16 */
			SPI_CR1_MSTR |
			/* SPI_CR1_CPOL | */ // = 0
			SPI_CR1_CPHA | // = 1
			0,
		.cr2 = SPI_CR2_16BIT_MODE
	},
	.direct_io = {
		TLE6240_DIRECT_IO
	},
	.reset = {.port = TLE6240_RESET_PORT, .pad = TLE6240_RESET_PAD}
};
#endif /* (BOARD_TLE6240_COUNT > 0) */

#if (BOARD_MC33972_COUNT > 0)
struct mc33972_config mc33972 = {
	.spi_bus = NULL /* TODO software lookup &SPID4 */,
	.spi_config = {
		.circular = false,
		.end_cb = NULL,
		.ssport = NULL,
		.sspad = 0,
		.cr1 =
			SPI_CR1_24BIT_MODE |
			SPI_CR1_SSM |
			SPI_CR1_SSI |
			/* SPI_CR1_LSBFIRST | */
			((3 << SPI_CR1_BR_Pos) & SPI_CR1_BR) |	/* div = 16 */
			SPI_CR1_MSTR |
			/* SPI_CR1_CPOL | */ /* = 0 */
			SPI_CR1_CPHA | /* = 1 */
			0,
		.cr2 = SPI_CR2_24BIT_MODE
	},
};
#endif /* (BOARD_MC33972_COUNT > 0) */

#if (BOARD_TLE8888_COUNT > 0)
struct tle8888_config tle8888_cfg = {
	.spi_bus = NULL,
	.spi_config = {
		.circular = false,
		.end_cb = NULL,
		.ssport = NULL,
		.sspad = 0,
		.cr1 =
			SPI_CR1_16BIT_MODE |
			SPI_CR1_SSM |
			SPI_CR1_SSI |
			SPI_CR1_LSBFIRST |	//LSB first
			((3 << SPI_CR1_BR_Pos) & SPI_CR1_BR) |	// div = 16
			SPI_CR1_MSTR |
			SPI_CR1_CPHA |
			0,
		.cr2 = SPI_CR2_16BIT_MODE
	},
	.reset =  {.port = NULL,	.pad = 0},
	.direct_gpio = {
		/* IN1..4 -> OUT1..OUT4 (Injectors) */
		[0] = {.port = GPIOE,	.pad = 14},
		[1] = {.port = GPIOE,	.pad = 13},
		[2] = {.port = GPIOE,	.pad = 12},
		[3] = {.port = GPIOE,	.pad = 11},
		/* IN5..8 -> IGN1..IGN4 (Ignotors) */
		/* Not used */
		[4] = {.port = NULL,	.pad = 0},
		[5] = {.port = NULL,	.pad = 0},
		[6] = {.port = NULL,	.pad = 0},
		[7] = {.port = NULL,	.pad = 0},
		/* Remapable IN9..12 */
		[8] = {.port = GPIOE,	.pad = 10},
		[9] = {.port = GPIOE,	.pad = 9},
		[10] = {.port = GPIOE,	.pad = 8},
		[11] = {.port = GPIOE,	.pad = 7},
	},
	.direct_maps = {
		[0] = {.output =  5},
		[1] = {.output =  6},
		[2] = {.output = 21},
		[3] = {.output = 22},
	},
	.ign_en =  {.port = GPIOD,	.pad = 10},
	.inj_en =  {.port = GPIOD,	.pad = 11},
	.mode = TL_AUTO,
	.stepper = false
};
#endif

#if (BOARD_DRV8860_COUNT > 0)
struct drv8860_config drv8860 = {
	.spi_bus = NULL /* TODO software lookup &SPID4 */,
	.spi_config = {
		.circular = false,
		.end_cb = NULL,
		.ssport = NULL,
		.sspad = 0,
		.cr1 =
			SPI_CR1_16BIT_MODE |
			SPI_CR1_SSM |
			SPI_CR1_SSI |
			((7 << SPI_CR1_BR_Pos) & SPI_CR1_BR) |	/* div = 32 */
			SPI_CR1_MSTR |
			SPI_CR1_CPOL |
			0,
		.cr2 = SPI_CR2_16BIT_MODE
	},
	.reset = {.port = DRV8860_RESET_PORT, .pad = DRV8860_RESET_PAD}
};
#endif /* (BOARD_DRV8860_COUNT > 0) */

void initSmartGpio() {
#if (BOARD_EXT_GPIOCHIPS > 0)
	startSmartCsPins();
#endif /* BOARD_EXT_GPIOCHIPS */
	int ret = -1;

#if (BOARD_TLE6240_COUNT > 0)
	if (engineConfiguration->tle6240_cs != GPIO_UNASSIGNED) {
		tle6240.spi_config.ssport = getHwPort("tle6240 CS", engineConfiguration->tle6240_cs);
		tle6240.spi_config.sspad = getHwPin("tle6240 CS", engineConfiguration->tle6240_cs);
		tle6240.spi_bus = getSpiDevice(engineConfiguration->tle6240spiDevice);
		ret = tle6240_add(0, &tle6240);
	} else {
		ret = -1;
	}
	if (ret < 0)
		/* whenever chip is disabled or error returned - occupy its gpio range */
		gpiochip_use_gpio_base(TLE6240_OUTPUTS);
#endif /* (BOARD_TLE6240_COUNT > 0) */

#if (BOARD_MC33972_COUNT > 0)
	if (engineConfiguration->mc33972_cs != GPIO_UNASSIGNED) {
		// todo: reuse initSpiCs method?
		mc33972.spi_config.ssport = getHwPort("mc33972 CS", engineConfiguration->mc33972_cs);
		mc33972.spi_config.sspad = getHwPin("mc33972 CS", engineConfiguration->mc33972_cs);
		mc33972.spi_bus = getSpiDevice(engineConfiguration->mc33972spiDevice);
		// todo: propogate 'basePinOffset' parameter
		ret = mc33972_add(0, &mc33972);
	} else {
		ret = -1;
	}
	if (ret < 0)
		/* whenever chip is disabled or error returned - occupy its gpio range */
		gpiochip_use_gpio_base(MC33972_INPUTS);
#endif /* (BOARD_MC33972_COUNT > 0) */

#if (BOARD_TLE8888_COUNT > 0)
	if (engineConfiguration->tle8888_cs != GPIO_UNASSIGNED) {
		// SPI pins are enabled in initSpiModules()

		// todo: reuse initSpiCs method?
		tle8888_cfg.spi_config.ssport = getHwPort("tle8888 CS", engineConfiguration->tle8888_cs);
		tle8888_cfg.spi_config.sspad = getHwPin("tle8888 CS", engineConfiguration->tle8888_cs);
		tle8888_cfg.spi_bus = getSpiDevice(engineConfiguration->tle8888spiDevice);

		tle8888_cfg.mode = engineConfiguration->tle8888mode;
		tle8888_cfg.stepper = engineConfiguration->useTLE8888_stepper;

		/* spi_bus == null checked in _add function */
		ret = tle8888_add(0, &tle8888_cfg);

		efiAssertVoid(OBD_PCM_Processor_Fault, ret == TLE8888_PIN_1, "tle8888");
	} else {
		ret = -1;
	}
	if (ret < 0)
		/* whenever chip is disabled or error returned - occupy its gpio range */
		gpiochip_use_gpio_base(TLE8888_SIGNALS);
#endif /* (BOARD_TLE8888_COUNT > 0) */

#if (BOARD_DRV8860_COUNT > 0)
	if (engineConfiguration->drv8860_cs != GPIO_UNASSIGNED) {
		drv8860.spi_config.ssport = getHwPort("drv8860 CS", engineConfiguration->drv8860_cs);
		drv8860.spi_config.sspad = getHwPin("drv8860 CS", engineConfiguration->drv8860_cs);
		drv8860.spi_bus = getSpiDevice(engineConfiguration->drv8860spiDevice);
		ret = drv8860_add(0, &drv8860);

		efiAssertVoid(OBD_PCM_Processor_Fault, ret == DRV8860_PIN_1, "drv8860");
	} else {
		ret = -1;
	}
	if (ret < 0)
		/* whenever chip is disabled or error returned - occupy its gpio range */
		gpiochip_use_gpio_base(DRV8860_OUTPUTS);
#endif /* (BOARD_DRV8860_COUNT > 0) */

#if (BOARD_EXT_GPIOCHIPS > 0)
	/* external chip init */
	gpiochips_init();
#endif /* (BOARD_EXT_GPIOCHIPS > 0) */
}

#if (BOARD_EXT_GPIOCHIPS > 0)
void stopSmartCsPins() {
#if (BOARD_TLE8888_COUNT > 0)
	brain_pin_markUnused(activeConfiguration.tle8888_cs);
#endif /* BOARD_TLE8888_COUNT */
#if (BOARD_TLE6240_COUNT > 0)
	brain_pin_markUnused(activeConfiguration.tle6240_cs);
#endif /* BOARD_TLE6240_COUNT */
#if (BOARD_MC33972_COUNT > 0)
	brain_pin_markUnused(activeConfiguration.mc33972_cs);
#endif /* BOARD_MC33972_COUNT */
#if (BOARD_DRV8860_COUNT > 0)
	brain_pin_markUnused(activeConfiguration.drv8860_cs);
#endif /* BOARD_DRV8860_COUNT */
}

void startSmartCsPins() {
#if (BOARD_TLE8888_COUNT > 0)
	tle8888Cs.initPin("tle8888 CS", engineConfiguration->tle8888_cs,
				&engineConfiguration->tle8888_csPinMode);
	tle8888Cs.setValue(true);
#endif /* BOARD_TLE8888_COUNT */
#if (BOARD_TLE6240_COUNT > 0)
	tle6240Cs.initPin("tle6240 CS", engineConfiguration->tle6240_cs,
				&engineConfiguration->tle6240_csPinMode);
	tle6240Cs.setValue(true);
#endif /* BOARD_TLE6240_COUNT */
#if (BOARD_MC33972_COUNT > 0)
	mc33972Cs.initPin("mc33972 CS", engineConfiguration->mc33972_cs,
				&engineConfiguration->mc33972_csPinMode);
	mc33972Cs.setValue(true);
#endif /* BOARD_MC33972_COUNT */
#if (BOARD_DRV8860_COUNT > 0)
	drv8860Cs.initPin("drv8860 CS", engineConfiguration->drv8860_cs,
				&engineConfiguration->drv8860_csPinMode);
	drv8860Cs.setValue(true);
#endif /* BOARD_DRV8860_COUNT */
}
#endif /* (BOARD_EXT_GPIOCHIPS > 0) */

#endif /* EFI_PROD_CODE */
