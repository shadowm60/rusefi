

#include "engine_test_helper.h"

TEST(hardware, reinit) {
	WITH_ENGINE_TEST_HELPER(MIATA_NA6_MAP);

	ButtonDebounce::stopConfigurationList();
	ButtonDebounce::startConfigurationList();

	ButtonDebounce::stopConfigurationList();
	ButtonDebounce::startConfigurationList();


	resetConfigurationExt(nullptr, nullptr, DODGE_NEON_1995 PASS_ENGINE_PARAMETER_SUFFIX);
	resetConfigurationExt(nullptr, nullptr, MIATA_NA6_MAP PASS_ENGINE_PARAMETER_SUFFIX);
}
