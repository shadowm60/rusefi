#include "engine_test_helper.h"
#include "timer.h"


TEST(util, timer) {
	WITH_ENGINE_TEST_HELPER(TEST_ENGINE);
	Timer timer;
	ASSERT_TRUE(timer.hasElapsedSec(3));
	timer.reset();
	ASSERT_FALSE(timer.hasElapsedSec(3));

	eth.moveTimeForwardSec(4);
	ASSERT_TRUE(timer.hasElapsedSec(3));
}
