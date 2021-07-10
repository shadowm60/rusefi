/*
 * global_execution_queue.cpp
 *
 * @date Dec 8, 2018
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "global_execution_queue.h"

bool_t debugSignalExecutor = false;

TestExecutor::~TestExecutor() {
	// Flush the queue and reset all scheduling_s at the end of a test's execution
	clear();
}

void TestExecutor::scheduleForLater(scheduling_s *scheduling, int delayUs, action_s action) {
	if (debugSignalExecutor) {
		printf("scheduleTask %d\r\n", delayUs);
	}

	if (m_mockExecutor) {
		m_mockExecutor->scheduleForLater(scheduling, delayUs, action);
		return;
	}

	scheduleByTimestamp(scheduling, getTimeNowUs() + delayUs, action);
}

int TestExecutor::executeAll(efitime_t now) {
	return schedulingQueue.executeAll(now);
}

void TestExecutor::clear() {
	schedulingQueue.clear();
}

int TestExecutor::size() {
	return schedulingQueue.size();
}

scheduling_s* TestExecutor::getHead() {
	return schedulingQueue.getHead();
}

scheduling_s* TestExecutor::getForUnitTest(int index) {
	return schedulingQueue.getElementAtIndexForUnitText(index);
}

void TestExecutor::scheduleByTimestamp(scheduling_s *scheduling, efitimeus_t timeUs, action_s action) {
	if (debugSignalExecutor) {
		printf("scheduleByTime %d\r\n", timeUs);
	}

	if (m_mockExecutor) {
		m_mockExecutor->scheduleByTimestamp(scheduling, timeUs, action);
		return;
	}

	schedulingQueue.insertTask(scheduling, timeUs, action);
}

void TestExecutor::scheduleByTimestampNt(scheduling_s* scheduling, efitick_t timeNt, action_s action) {
	if (m_mockExecutor) {
		m_mockExecutor->scheduleByTimestampNt(scheduling, timeNt, action);
		return;
	}

	scheduleByTimestamp(scheduling, NT2US(timeNt), action);
}

void TestExecutor::setMockExecutor(ExecutorInterface* exec) {
	m_mockExecutor = exec;
}
