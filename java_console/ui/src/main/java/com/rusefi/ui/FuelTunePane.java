package com.rusefi.ui;

import com.opensr5.Logger;
import com.romraider.Settings;
import com.romraider.maps.Scale;
import com.romraider.maps.Table;
import com.romraider.maps.Table3D;
import com.romraider.xml.RomAttributeParser;
import com.rusefi.BinarySearch;
import com.opensr5.ConfigurationImage;
import com.rusefi.FileLog;
import com.rusefi.UploadChanges;
import com.rusefi.autotune.FuelAutoTune;
import com.rusefi.autotune.Result;
import com.rusefi.autotune.AfrDataPoint;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.config.generated.Fields;
import com.rusefi.core.ISensorCentral;
import com.rusefi.core.Sensor;
import com.rusefi.core.SensorCentral;
import com.rusefi.ui.storage.Node;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

/**
 * Andrey Belomutskiy, (c) 2013-2020
 * 1/9/2016
 *
 * @see FuelAutoTune
 */
public class FuelTunePane {
    private final JPanel content = new JPanel(new BorderLayout());

    private final static int veLoadOffset = Fields.VETABLE.getOffset() + Fields.FUEL_RPM_COUNT * Fields.FUEL_LOAD_COUNT * 4;
    private final static int veRpmOffset = Fields.VETABLE.getOffset() + Fields.FUEL_RPM_COUNT * Fields.FUEL_LOAD_COUNT * 4 + Fields.FUEL_LOAD_COUNT * 4;


    private final List<FuelDataPoint> incomingDataPoints = new ArrayList<>();
    private final double veLoadBins[] = new double[Fields.FUEL_LOAD_COUNT];
    private final double veRpmBins[] = new double[Fields.FUEL_RPM_COUNT];
    private final Table3D veTable = new Table3D();
    private final Table3D changeMap = new Table3D();
    private final JButton upload = new JButton("Upload");
    private final JCheckBox collect = new JCheckBox("enable");
    private final JButton clean = new JButton("clear");
    private final UIContext uiContext;
    private byte[] newVeMap;
    private DataOutputStream dos;

    public FuelTunePane(UIContext uiContext, Node config) {
        this.uiContext = uiContext;
        final JLabel incomingBufferSize = new JLabel();

        JButton runLogic = new JButton("one iteration");
        runLogic.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                doJob();
            }
        });
        upload.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                uploadCurrentResult();
            }
        });
        clean.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                doClean();
            }
        });

        collect.setSelected(false);
        JPanel topPanel = new JPanel(new FlowLayout());
        topPanel.add(collect);
        topPanel.add(clean);
        topPanel.add(incomingBufferSize);
        topPanel.add(runLogic);
        topPanel.add(upload);

        Timer runTime = new Timer(1000, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                boolean runJob;
                synchronized (incomingDataPoints) {
                    runJob = incomingDataPoints.size() > 50;
                }
                if (runJob) {
                    doJob();
                    uploadCurrentResult();
                }
            }
        });
        runTime.start();


        // todo: records based on change, not based on timer
        Timer timer = new Timer(300, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int size;
                synchronized (incomingDataPoints) {
                    size = incomingDataPoints.size();
                }
                incomingBufferSize.setText(size + " records in buffer");
            }
        });
        timer.start();

        upload.setEnabled(false);
        content.add(topPanel, BorderLayout.NORTH);

        JPanel rightPanel = new JPanel(new GridLayout(2, 1));
        rightPanel.add(changeMap);
        GaugesGrid grid = new GaugesGrid(1, 3);
        rightPanel.add(grid.panel);

        grid.panel.add(GaugesGridElement.read(uiContext, config.getChild("1"), Sensor.RPM));
        grid.panel.add(GaugesGridElement.read(uiContext, config.getChild("2"), Sensor.Lambda));

        JPanel middlePanel = new JPanel(new GridLayout(1, 2));
        middlePanel.add(veTable);
        middlePanel.add(rightPanel);

        content.add(middlePanel, BorderLayout.CENTER);
        initTable(veTable);
        initTable(changeMap);
    }

    private void uploadCurrentResult() {
        byte[] newVeMap = FuelTunePane.this.newVeMap;
        BinaryProtocol bp = uiContext.getLinkManager().getCurrentStreamState();
        if (newVeMap == null || bp == null)
            return;
        ConfigurationImage ci = bp.getControllerConfiguration().clone();
        System.arraycopy(newVeMap, 0, ci.getContent(), Fields.VETABLE.getOffset(), newVeMap.length);
        Runnable afterBurn = new Runnable() {
            @Override
            public void run() {
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                        // let's clean so that we collect fresh datapoints based on new settings
                        doClean();
                        reloadVeTable();
                    }
                });
            }
        };
        UploadChanges.scheduleUpload(uiContext, ci, afterBurn);
    }

    private void doClean() {
        synchronized (incomingDataPoints) {
            incomingDataPoints.clear();
        }
    }

    private static void loadData(Table table, byte[] content, int offset) {
        table.reset();
        table.setStorageAddress(offset);
        table.setStorageType(Settings.STORAGE_TYPE_FLOAT);
        table.populateTable(content, 0);
        table.drawTable();
    }

    private void initTable(Table3D table) {
        // todo: which one is which?
        table.setSizeX(Fields.FUEL_LOAD_COUNT);
        table.setSizeY(Fields.FUEL_RPM_COUNT);
        table.getXAxis().setDataSize(Fields.FUEL_LOAD_COUNT);
        table.getYAxis().setDataSize(Fields.FUEL_RPM_COUNT);

        table.getXAxis().setAxisParent(table);
        table.getYAxis().setAxisParent(table);

        table.setBorder(BorderFactory.createLineBorder(Color.red));
        table.addScale(new Scale());
        table.getXAxis().addScale(new Scale());
        table.getYAxis().addScale(new Scale());
    }

    private void doJob() {
        double veTable[][] = new double[Fields.FUEL_LOAD_COUNT][Fields.FUEL_RPM_COUNT];
        loadMap(veTable, Fields.VETABLE.getOffset());
        logMap("source", veTable);

        List<AfrDataPoint> data = new ArrayList<>();
        synchronized (incomingDataPoints) {
            for (FuelDataPoint point : incomingDataPoints)
                data.add(point.asDataOnline());
            incomingDataPoints.clear();
        }
        writeDataPoints(data);

        // todo: move this away from AWT thread
        Result a = FuelAutoTune.INSTANCE.process(false, data, 0.1, 14.7, veTable);

        double[][] result = a.getKgbcRES();
        logMap("result", result);
        newVeMap = toByteArray(result);

        loadData(changeMap, newVeMap, 0);
        upload.setEnabled(true);
    }

    private void writeDataPoints(List<AfrDataPoint> data) {
        DataOutputStream dos = getTuneLogStream();
        if (dos == null)
            return;
        try {
            dos.writeBytes("Running with " + data.size() + " points\r\n");
            dos.writeBytes("afr\tRPM\tload\r\n");
            for (AfrDataPoint point : data)
                dos.writeBytes(point.getAfr() +"\t" + point.getRpm() + "\t" + point.getEngineLoad() + "\r\n");

        } catch (IOException e) {
            FileLog.MAIN.logLine("Error writing auto-tune log");
        }
    }

    private void logMap(String msg, double[][] table) {
        DataOutputStream dos = getTuneLogStream();
        if (dos == null)
            return;
        try {
            dos.writeBytes(new Date() + ": " + msg + "\r\n");

            for (int rpmIndex = 0; rpmIndex < Fields.FUEL_RPM_COUNT; rpmIndex++) {
                dos.writeChar('\t');
                dos.writeBytes(Double.toString(veRpmBins[rpmIndex]));
            }
            dos.writeBytes("\r\n");

            for (int loadIndex = 0; loadIndex < Fields.FUEL_LOAD_COUNT; loadIndex++) {
                dos.writeBytes(Double.toString(veLoadBins[loadIndex]));
                for (int rpmIndex = 0; rpmIndex < Fields.FUEL_RPM_COUNT; rpmIndex++) {
                    dos.writeChar('\t');
                    double v = table[loadIndex][rpmIndex];
                    dos.writeBytes(Double.toString(v));
                }
                dos.writeBytes("\r\n");
            }
            dos.flush();
        } catch (IOException e) {
            FileLog.MAIN.logLine("Error writing auto-tune log");
        }
    }

    private DataOutputStream getTuneLogStream() {
        if (dos == null) {
            String fileName = Logger.DIR + "tune_" + Logger.getDate() + ".txt";
            try {
                dos = new DataOutputStream(new FileOutputStream(fileName));
            } catch (FileNotFoundException e) {
                FileLog.MAIN.logLine("Error creating " + fileName + ":" + e);
            }
        }
        return dos;
    }

    private byte[] toByteArray(double[][] output) {
        try {
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            for (int loadIndex = 0; loadIndex < Fields.FUEL_LOAD_COUNT; loadIndex++) {
                for (int rpmIndex = 0; rpmIndex < Fields.FUEL_RPM_COUNT; rpmIndex++) {
                    byte[] b4 = RomAttributeParser.floatToByte((float)output[loadIndex][rpmIndex], Settings.ENDIAN_BIG);
                    baos.write(b4);
                }
            }
            return baos.toByteArray();
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }

    public void showContent() {
        final ISensorCentral sc = SensorCentral.getInstance();
        sc.addListener(Sensor.RPM, new SensorCentral.SensorListener() {
            @Override
            public void onSensorUpdate(double value) {
                if (!collect.isSelected())
                    return;
                int rpm = (int) value;
                double engineLoad = sc.getValue(Sensor.MAP);
                double afr = sc.getValue(Sensor.Lambda);
                // todo: add UI for pre-conditions
                double deltaTps = sc.getValue(Sensor.deltaTps);
                double clt = sc.getValue(Sensor.CLT);
//                if (clt < 80)
//                    return;
                FuelDataPoint newPoint = new FuelDataPoint(rpm, engineLoad, afr);
                synchronized (incomingDataPoints) {
                    incomingDataPoints.add(newPoint);
                }
            }
        });

        loadArray(veLoadBins, veLoadOffset);
        loadArray(veRpmBins, veRpmOffset);

        byte[] content = reloadVeTable();

        loadData(changeMap.getXAxis(), content, veRpmOffset);
        loadData(changeMap.getYAxis(), content, veLoadOffset);
    }

    private byte[] reloadVeTable() {
        BinaryProtocol bp = uiContext.getLinkManager().getCurrentStreamState();

        byte[] content = bp.getControllerConfiguration().getContent();
        loadData(veTable.getXAxis(), content, veRpmOffset);
        loadData(veTable.getYAxis(), content, veLoadOffset);
        loadData(veTable, content, Fields.VETABLE.getOffset());
        return content;
    }

    private void loadMap(double[][] map, int offset) {
        for (int engineLoadIndex = 0; engineLoadIndex < map.length; engineLoadIndex++) {
            loadArray(map[engineLoadIndex], offset + engineLoadIndex * 4 * Fields.FUEL_RPM_COUNT);
        }
    }

    private void loadArray(double[] array, int offset) {
        BinaryProtocol bp = uiContext.getLinkManager().getCurrentStreamState();
        if (bp == null) {
            FileLog.MAIN.logLine("bp not ready");
            return;
        }
        for (int i = 0; i < array.length; i++)
            array[i] = bp.getControllerConfiguration().getByteBuffer(offset + 4 * i, 4).getFloat();
        System.out.println("FuelTunePane: Loaded " + Arrays.toString(array));
    }

    public Component getContent() {
        return content;
    }

    private class FuelDataPoint {
        private final int rpm;
        private final double engineLoad;
        private final double afr;
        private final int rpmIndex;
        private final int engineLoadIndex;

        public FuelDataPoint(int rpm, double engineLoad, double afr) {
            this.rpm = rpm;
            this.engineLoad = engineLoad;
            this.afr = afr;
            // too low values are returning '-1' indeces
            rpmIndex = Math.max(0, BinarySearch.binarySearch(rpm, veRpmBins));
            engineLoadIndex = Math.max(0, BinarySearch.binarySearch(engineLoad, veLoadBins));
        }

        public AfrDataPoint asDataOnline() {
            return new AfrDataPoint(afr, rpmIndex, engineLoadIndex, rpm, engineLoad);
        }
    }
}
