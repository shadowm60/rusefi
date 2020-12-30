package com.rusefi;

import com.rusefi.config.generated.Fields;
import com.rusefi.core.Sensor;
import com.rusefi.core.SensorCentral;
import com.rusefi.functional_tests.EcuTestHelper;
import org.junit.Ignore;
import org.junit.Test;

import static com.rusefi.IoUtil.getDisableCommand;
import static com.rusefi.IoUtil.getEnableCommand;
import static com.rusefi.binaryprotocol.BinaryProtocol.sleep;
import static com.rusefi.config.generated.Fields.*;

/**
 * This test relies on jumpers connecting physical pins on Discovery:
 * PD1<>PC6
 * PD2<>PA5
 */

public class PwmHardwareTest extends RusefiTestBase {
    @Override
    boolean needsHardwareTriggerInput() {
        // This test uses hardware trigger input!
        return true;
    }

    private static final int FREQUENCY = 160;

    @Test
    public void testIdlePin() {
        ecu.setEngineType(ET_FRANKENSO_MIATA_NA6);
        ecu.sendCommand(getDisableCommand(Fields.CMD_SELF_STIMULATION));
        ecu.changeRpm(1000);

        ecu.sendCommand(CMD_TRIGGER_SIMULATOR_PIN + " 0 none");
        ecu.sendCommand(CMD_TRIGGER_SIMULATOR_PIN + " 1 none");
        ecu.sendCommand(CMD_IDLE_PIN + " PD2");

        /* wasted two wire, so we really need 1..3 to be none */
        ecu.sendCommand(CMD_IGNITION_PIN + " 1 none");
        ecu.sendCommand(CMD_IGNITION_PIN + " 2 none");
        ecu.sendCommand(CMD_IGNITION_PIN + " 3 none");

        ecu.sendCommand(CMD_TRIGGER_PIN + " 1 PA8");

        /* DBG_LOGIC_ANALYZER */
        ecu.sendCommand("set debug_mode " + DBG_DBG_LOGIC_ANALYZER);

        /* 160 Hz */
        ecu.sendCommand("set idle_solenoid_freq " + FREQUENCY);

        /* save these for last to ensure logic is started */
        ecu.sendCommand(CMD_LOGIC_PIN + " 0 PA5");
        ecu.sendCommand(CMD_WRITECONFIG);
        sleep(2 * Timeouts.SECOND);
        ecu.sendCommand(getEnableCommand(Fields.CMD_SELF_STIMULATION));

        sleep(2 * Timeouts.SECOND);

        /* +-1% is still acceptable */
        EcuTestHelper.assertEquals("Idle PWM freq", FREQUENCY, SensorCentral.getInstance().getValue(Sensor.debugIntField1), 0.01);
    }
}