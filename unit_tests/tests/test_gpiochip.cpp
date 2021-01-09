/**
 * @file	test_gpiochip.cpp
 *
 * @date Mar 12, 2019
 */

#include "unit_test_framework.h"
#include "global.h"
#include "gpio/gpio_ext.h"

using ::testing::_;

static int testchip_readPad(void *data, unsigned int pin)
{
	if (pin & 0x01)
		return 1;
	return 0;
}

static int io_state = 0;

static int testchip_writePad(void *data, unsigned int pin, int value)
{
	if (value)
		io_state |=  (1 << value);
	else
		io_state &= ~(1 << value);

	return 0;
}

static int initcalls = 0;

static int testchip_init(void *data)
{
	initcalls++;

	return 0;
}

static int calls_to_failed_chip = 0;
static int testchip_failed_writePad(void *data, unsigned int pin, int value)
{
	calls_to_failed_chip++;
	return 0;
}

static int testchip_failed_init(void *data)
{
	return -1;
}

/* invalid chip */
struct gpiochip_ops testchip0 = {
	/*.setPadMode	=*/ NULL,
	/*.writePad		=*/ NULL,
	/*.readPad		=*/ NULL,
	/*.getDiag		=*/ NULL,
	/*.init			=*/ testchip_init,
	/*.deinit 		=*/ NULL,
};

/* Input only chip */
struct gpiochip_ops testchip1 = {
	/*.setPadMode	=*/ NULL,
	/*.writePad		=*/ NULL,
	/*.readPad		=*/ testchip_readPad,
	/*.getDiag		=*/ NULL,
	/*.init			=*/ testchip_init,
	/*.deinit 		=*/ NULL,
};

/* Input only chip */
struct gpiochip_ops testchip2 = {
	/*.setPadMode	=*/ NULL,
	/*.writePad		=*/ testchip_writePad,
	/*.readPad		=*/ NULL,
	/*.getDiag		=*/ NULL,
	/*.init			=*/ testchip_init,
	/*.deinit 		=*/ NULL,
};

/* testchi[ failed to init */
struct gpiochip_ops testchip3 = {
	/*.setPadMode	=*/ NULL,
	/*.writePad		=*/ testchip_failed_writePad,
	/*.readPad		=*/ NULL,
	/*.getDiag		=*/ NULL,
	/*.init			=*/ testchip_failed_init,
	/*.deinit 		=*/ NULL,
};

TEST(gpioext, testGpioExt) {
	int ret;
	int chip1_base, chip2_base, chip3_base;

	printf("====================================================================================== testGpioExt\r\n");

	/* should fail to register chip with no readPad and writePad */
	EXPECT_FALSE(gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1), "invalid", &testchip0, 16, NULL) > 0);

	/* should fail to register chip with zero gpios */
	EXPECT_FALSE(gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1), "invalid", &testchip1, 0, NULL) > 0);

	/* should fail to register chip with base overlapig on-chip gpios */
	EXPECT_FALSE(gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST - 1), "invalid", &testchip1, 0, NULL) > 0);

	chip1_base = gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1), "input only", &testchip1, 16, NULL);
	EXPECT_TRUE(chip1_base > 0);

	EXPECT_EQ(16, gpiochips_get_total_pins());

	/* should fail to register chip overlapping other one */
	EXPECT_FALSE(gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1 + 15), "output only", &testchip2, 16, NULL) > 0);

	chip2_base = gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1 + 16), "output only", &testchip2, 16, NULL);
	EXPECT_TRUE(chip2_base > 0);

	/* this chip will fail to init, but should be registered without errors */
	chip3_base = gpiochip_register((brain_pin_e)(BRAIN_PIN_ONCHIP_LAST + 1 + 16 + 16), "failed chip", &testchip3, 16, NULL);
	EXPECT_TRUE(chip2_base > 0);

	EXPECT_EQ(48, gpiochips_get_total_pins());

	/* init 3 chips, one will fail */
	ret = gpiochips_init();
	EXPECT_EQ(32, ret);

	/* two drivers should be inited */
	EXPECT_EQ(2, initcalls);

	/* gpio reads */
	EXPECT_TRUE(gpiochips_readPad((brain_pin_e)(chip1_base + 0)) == 0);
	EXPECT_TRUE(gpiochips_readPad((brain_pin_e)(chip1_base + 1)) != 0);

	/* gpio write */
	gpiochips_writePad((brain_pin_e)(chip2_base + 0), 0);
	gpiochips_writePad((brain_pin_e)(chip2_base + 1), 1);
	EXPECT_EQ(0x02, io_state);

	/* try to access failed chip */
	EXPECT_FALSE(gpiochips_writePad((brain_pin_e)(chip3_base + 0), 0) >= 0);
	EXPECT_FALSE(gpiochips_writePad((brain_pin_e)(chip3_base + 1), 1) >= 0);
	EXPECT_EQ(0, calls_to_failed_chip);

	/* read/write outside range */
	EXPECT_TRUE(gpiochips_readPad((brain_pin_e)(chip1_base - 1)) < 0);
	EXPECT_TRUE(gpiochips_writePad((brain_pin_e)(chip1_base - 1), 1) < 0);

	EXPECT_TRUE(gpiochips_readPad((brain_pin_e)(chip3_base + 16)) < 0);
	EXPECT_TRUE(gpiochips_writePad((brain_pin_e)(chip3_base + 16), 1) < 0);

}
