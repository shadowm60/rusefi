/**
 * @file        lps25.h
 * @brief       Driver for the ST LPS25HB pressure sensor
 *
 * @date February 6, 2020
 * @author Matthew Kennedy, (c) 2020
 */

#pragma once

#if EFI_PROD_CODE

#include "i2c_bb.h"

#include "expected.h"

class Lps25 {
public:
	// Returns true if the sensor was initialized successfully.
	bool init(brain_pin_e scl, brain_pin_e sda);

	expected<float> readPressureKpa();

private:
	BitbangI2c m_i2c;
	bool m_hasInit = false;
};

#endif // EFI_PROD_CODE
