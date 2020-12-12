package com.rusefi;

import com.rusefi.config.generated.Fields;
import com.rusefi.trigger.WaveState;
import org.jetbrains.annotations.NotNull;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

class TriggerWheelInfo {
    final int id;
    final double tdcPosition;
    final String triggerName;
    final List<WaveState> waves;
    final List<TriggerSignal> signals;

    public TriggerWheelInfo(int id, double tdcPosition, String triggerName, List<WaveState> waves, List<TriggerSignal> signals) {
        this.id = id;
        this.tdcPosition = tdcPosition;
        this.triggerName = triggerName;
        this.waves = waves;
        this.signals = signals;
    }

    static TriggerWheelInfo readTriggerWheelInfo(String line, BufferedReader reader) throws IOException {
        String[] tokens = line.split(" ");
        String idStr = tokens[1];
        int eventCount = Integer.parseInt(tokens[2]);
        String triggerName = tokens[3];
        System.out.println("Processing " + line + " " + idStr);

        int id = Integer.parseInt(idStr);
        double tdcPosition = Double.parseDouble(tokens[4]);

        System.out.println("id=" + id + ", count=" + eventCount + ", name=" + triggerName);

        List<TriggerSignal> signals = TriggerImage.readSignals(reader, eventCount);

        List<WaveState> waves = TriggerImage.convertSignalsToWaves(signals);

        return new TriggerWheelInfo(id, tdcPosition, triggerName, waves, signals);
    }

    @NotNull
    List<TriggerSignal> getFirstWheeTriggerSignals() {
        List<TriggerSignal> firstWheel = getTriggerSignals(0);
        if (isFirstCrankBased()) {
            return takeFirstHalf(firstWheel);
        } else {
            return compressAngle(firstWheel);
        }
    }

    public double getTdcPosition() {
        return tdcPosition;
    }

    @NotNull
    private List<TriggerSignal> getTriggerSignals(int index) {
        return signals.stream().filter(signal -> signal.waveIndex == index).collect(Collectors.toList());
    }

    @NotNull
    private List<TriggerSignal> takeFirstHalf(List<TriggerSignal> wheel) {
        return wheel.stream().filter(triggerSignal -> triggerSignal.angle < 360).collect(Collectors.toList());
    }

    /**
     * this is about converting 720 cycle of crank wheel shape into normal 360 circle range
     */
    @NotNull
    private static List<TriggerSignal> compressAngle(List<TriggerSignal> wheel) {
        return wheel.stream().map(triggerSignal -> new TriggerSignal(triggerSignal.waveIndex, triggerSignal.state, triggerSignal.angle / 2)).collect(Collectors.toList());
    }

    public List<TriggerSignal> getSecondWheeTriggerSignals() {
        List<TriggerSignal> secondWheel = getTriggerSignals(1);
        if (isSecondCamBased()) {
            return compressAngle(secondWheel);
        } else {
            return takeFirstHalf(secondWheel);
        }
    }

    private boolean isFirstCrankBased() {
        return id == Fields.TT_TT_GM_LS_24 ||
                id == Fields.TT_TT_HONDA_K_12_1 ||
                id == Fields.TT_TT_RENIX_44_2_2 ||
                id == Fields.TT_TT_RENIX_66_2_2_2 ||
                id == Fields.TT_TT_MIATA_VVT ||
                id == Fields.TT_TT_GM_7X;
    }

    private boolean isSecondCamBased() {
        return id == Fields.TT_TT_MAZDA_MIATA_NA ||
                id == Fields.TT_TT_MAZDA_DOHC_1_4 ||
                id == Fields.TT_TT_FORD_ASPIRE;
    }
}
