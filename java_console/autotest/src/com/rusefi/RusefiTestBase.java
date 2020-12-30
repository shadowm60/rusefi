package com.rusefi;

import com.rusefi.functional_tests.EcuTestHelper;
import org.junit.Before;

public class RusefiTestBase {
    protected EcuTestHelper ecu;

    boolean needsHardwareTriggerInput() {
        // Most tests do not, but some may need it
        return false;
    }

    @Before
    public void startUp() {
        ecu = EcuTestHelper.createInstance(needsHardwareTriggerInput());
    }
}
