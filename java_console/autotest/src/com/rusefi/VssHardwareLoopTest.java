package com.rusefi;

import com.rusefi.config.generated.Fields;
import com.rusefi.core.Sensor;
import com.rusefi.core.SensorCentral;
import com.rusefi.functional_tests.EcuTestHelper;
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
public class VssHardwareLoopTest {
    @Test
    public void test() {
        EcuTestHelper ecu = EcuTestHelper.createInstance(true);

        ecu.setEngineType(ET_FRANKENSO_MIATA_NA6);
        ecu.sendCommand(getDisableCommand(Fields.CMD_SELF_STIMULATION));
        ecu.changeRpm(1400);

        // moving second trigger to another pin
        ecu.sendCommand(CMD_TRIGGER_PIN + " 1 PA8");

        EcuTestHelper.assertEquals("VSS no input", 0, SensorCentral.getInstance().getValue(Sensor.VSS));

        // attaching VSS to trigger simulator since there is a jumper on test discovery
        ecu.sendCommand("set " + CMD_VSS_PIN + " pa5");

        sleep(2 * Timeouts.SECOND);

        EcuTestHelper.assertEquals("VSS with input", 3, SensorCentral.getInstance().getValue(Sensor.VSS));
        if (ControllerConnectorState.firmwareVersion == null)
            throw new IllegalStateException("firmwareVersion has not arrived");
    }

}
