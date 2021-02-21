package com.rusefi.proteus;

import com.rusefi.RusefiTestBase;
import com.rusefi.core.Sensor;
import com.rusefi.core.SensorCentral;
import org.junit.Test;

import static com.rusefi.config.generated.Fields.*;
import static com.rusefi.IoUtil.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/*
 This test requires a particular hardware setup connected to a Proteus board.
    - External power supplied to the 12v_IGN pin, nominal 12 volts (but 11-13 will work, see testVbatt).
        Go buy some cheapie $8 12v power supply, cut the barrel jack off, and crimp pins on.
    - A jumper wire from Ignition output 10 to Analog volt 2 (TPS, see testTpsAnalogInput)
 */
public class ProteusAnalogTest extends RusefiTestBase {
    @Test
    public void testVbatt() {
        double vbatt = SensorCentral.getInstance().getValue(Sensor.VBATT);

        // allow some tolerance for the supply voltage...
        assertTrue(vbatt > 11);
        assertTrue(vbatt < 13);
    }

    void setIdlePositionAndAssertTps(int idle, int expectedTps) {
        ecu.sendCommand("set idle_position " + idle);

        // wait a sec for sensors to update
        sleepSeconds(1);

        double actualTps = SensorCentral.getInstance().getValue(Sensor.TPS);

        // Accept up to 5% error - the PWM is a bit noisy, but it should be at least close
        assertEquals(expectedTps, actualTps, 5);
    }

    @Test
    public void testTpsAnalogInput() {
        ecu.setEngineType(ET_PROTEUS_ANALOG_PWM_TEST); // proteus analog input PWM tester
        ecu.changeRpm(1000);
        // engine has to be spinning for idle valve to work
        ecu.sendCommand(getEnableCommand(CMD_SELF_STIMULATION));

        // 0% duty -> failed TPS (voltage too low)
        setIdlePositionAndAssertTps(2, 0);

        // These should all be valid points
        setIdlePositionAndAssertTps(20, 0);
        setIdlePositionAndAssertTps(35, 25);
        setIdlePositionAndAssertTps(50, 50);
        setIdlePositionAndAssertTps(65, 75);
        setIdlePositionAndAssertTps(80, 100);

        // 100% duty -> failed TPS (voltage too high)
        setIdlePositionAndAssertTps(98, 0);
    }
}
