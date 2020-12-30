package com.rusefi.ui.livedocs;

import com.opensr5.ConfigurationImage;
import com.rusefi.autoupdate.AutoupdateUtil;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.config.Field;
import com.rusefi.config.generated.Fields;
import com.rusefi.core.Sensor;
import com.rusefi.core.SensorCentral;
import com.rusefi.ldmp.*;
import com.rusefi.ldmp.generated.*;
import com.opensr5.ini.DialogModel;
import com.opensr5.ini.IniFileModel;
import com.rusefi.ui.UIContext;
import com.rusefi.ui.livedocs.controls.Toolbox;
import com.rusefi.ui.widgets.DetachedSensor;
import net.miginfocom.swing.MigLayout;
import org.jetbrains.annotations.NotNull;
import org.putgemin.VerticalFlowLayout;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import static com.rusefi.config.Field.niceToString;

public class LiveDocPanel {

    private static final String CONSTRAINTS = "wrap, grow";
    private static final String LAYOUT = "gap 0, insets 0";
    // todo: replace magic hard-coded value with some relative number, maybe 1/3 of frame height or something?
    private static final int MAGIC_DETACHED_GAUGE_SIZE = 200;
    private static final int LIVE_DATA_PRECISION = 2;


    @NotNull
    public static JPanel createPanel(UIContext uiContext, String title, Request[] content) {
        return createPanel(uiContext, title, content, "", StateDictionary.NONE);
    }

    @NotNull
    static JPanel createPanel(UIContext uiContext, String title, Request[] content, String settingsInstancePrefix, final int defaultContextId) {
        LiveDataContext defaultContext = new LiveDataContext(defaultContextId);

        ActionPanel ap = createComponents(uiContext, title, content, settingsInstancePrefix, defaultContext);
        JPanel panel = ap.getPanel();

        LiveDocHolder holder = new LiveDocHolder(defaultContext, ap.getRefreshActions()) {
            @Override
            public boolean isVisible() {
                boolean isVisible = !panel.getVisibleRect().isEmpty();
                return isVisible && isRecursivelyVisible(panel);
            }
        };

        LiveDocsRegistry.INSTANCE.register(holder);
        return panel;
    }

    static boolean isRecursivelyVisible(Container c) {
        Container parent = c.getParent();
        return c.isVisible() && (parent == null || isRecursivelyVisible(parent));
    }

    private static ActionPanel createComponents(UIContext uiContext, String title, Request[] content, String settingsInstancePrefix, LiveDataContext defaultContext) {
        ActionPanel result = new ActionPanel(title);

        for (Request r : content) {
            if (r instanceof TextRequest) {
                TextRequest request = (TextRequest) r;
                if (request.isEol()) {
                    result.newLine();
                } else {
                    result.addControl(new JLabel(request.getValue().replaceAll("_", " ")));
                }
            } else if (r instanceof FieldRequest) {
                FieldRequest request = (FieldRequest) r;
                LiveDataContext context = getFieldContext(defaultContext, request.getStateContext());
                Field field = getField(defaultContext, request);
                JLabel label = new LessJumpyJLabel("*");
                label.setIcon(AutoupdateUtil.loadIcon("livedocs/variable.png"));
                label.setToolTipText("Variable " + field.getName());
                result.addControl(label);
                result.actionsListAdd(context, new RefreshActions() {
                    @Override
                    public void refresh(BinaryProtocol bp, byte[] response) {
                        Number fieldValue = field.getValue(new ConfigurationImage(response));
                        String value = Field.niceToString(fieldValue, LIVE_DATA_PRECISION);
                        label.setText(value);
                    }
                });
            } else if (r instanceof ConfigRequest) {
                ConfigRequest request = (ConfigRequest) r;
                Field field = Field.findField(Fields.VALUES, settingsInstancePrefix, request.getField());

                JLabel label = new LessJumpyJLabel("*");
                label.setIcon(AutoupdateUtil.loadIcon("livedocs/setting.png"));
                label.setToolTipText(getTooltipText(field.getName()));
                result.addControl(label);
                // todo: use different notification flow altogether since configuration has nothing to do with live data structures
                result.actionsListAdd(new LiveDataContext(Fields.LDS_ENGINE_STATE_INDEX), new RefreshActions() {
                    @Override
                    public void refresh(BinaryProtocol bp, byte[] response) {
                        double multiplier = 1; // todo: PROPER MULTIPLIER!!!
                        String value = field.getAnyValue(bp.getControllerConfiguration(), multiplier).toString();
                        label.setText(value);
                    }
                });
            } else if (r instanceof SensorRequest) {
                SensorRequest request = (SensorRequest) r;
                Sensor sensor = Sensor.find(request.getValue());
                JLabel label = new LessJumpyJLabel("*");
                label.setIcon(AutoupdateUtil.loadIcon("livedocs/gauge.png"));
                label.setToolTipText("Sensor " + request.getValue());
                label.addMouseListener(new MouseAdapter() {
                    @Override
                    public void mouseClicked(MouseEvent e) {
                        if (e.getClickCount() == 2 && !SwingUtilities.isRightMouseButton(e)) {
                            final DetachedSensor ds = new DetachedSensor(uiContext, sensor, MAGIC_DETACHED_GAUGE_SIZE);
                            ds.show(e);
                        }
                    }
                });
                result.addControl(label);
                SensorCentral.getInstance().addListener(sensor, value -> label.setText(niceToString(value, LIVE_DATA_PRECISION)));
            } else if (r instanceof IfRequest) {
                IfRequest request = (IfRequest) r;

                IfConditionPanel panel = createIfRequestPanel(uiContext, request, defaultContext);

                result.actionsListAddAll(panel.getActionsList());

                result.addControl(panel.getPanel());
            } else {
                throw new UnsupportedOperationException(r.toString());
            }
        }
        return result;
    }

    private static LiveDataContext getFieldContext(LiveDataContext defaultContext, String stateContext) {
        if (stateContext.isEmpty()) {
            return defaultContext;
        } else {
            String indexFieldName = StateDictionary.getContextIndexFieldName(stateContext);
            return StateDictionary.getStateContext(indexFieldName);
        }
    }

    private static Field getField(LiveDataContext defaultContext, FieldReference request) {
        Field[] context;
        if (request.getStateContext().isEmpty()) {
            context = StateDictionary.INSTANCE.getFields("create", defaultContext);
        } else {
            context = StateDictionary.INSTANCE.getValue(request.getStateContext());
        }
        return Field.findField(context, "", request.getField());
    }

    private static String getTooltipText(String configurationFieldName) {
        DialogModel.Field dialogField = IniFileModel.getInstance().getField(configurationFieldName);
        if (dialogField == null) {
            return "Configuration " + configurationFieldName;
        }
        return "Configuration " + dialogField.getUiName() + " (" + configurationFieldName + ")";
    }

    private static IfConditionPanel createIfRequestPanel(UIContext uiContext, IfRequest request, LiveDataContext defaultContext) {
        Field conditionField = getField(defaultContext, request);

        JPanel result = new JPanel(new VerticalFlowLayout());

        JLabel conditionLabel = new LessJumpyJLabel(request.getField());
        result.add(conditionLabel);


        ActionPanel trueAP = createComponents(uiContext, "", request.trueBlock.toArray(new Request[0]), "", defaultContext);
        ActionPanel falseAP = createComponents(uiContext, "", request.falseBlock.toArray(new Request[0]), "", defaultContext);

        result.add(trueAP.getPanel());
        result.add(falseAP.getPanel());

        RefreshActionsMap combined = trueAP.getRefreshActions();
        combined.addAll(falseAP.getRefreshActions());

        LiveDataContext context = getFieldContext(defaultContext, request.getStateContext());

        combined.put(context, new RefreshActions() {
            @Override
            public void refresh(BinaryProtocol bp, byte[] response) {
                int value = (int) conditionField.getValue(new ConfigurationImage(response)).intValue();
                conditionLabel.setText(request.getField() + " is " + (value == 1 ? "TRUE" : "FALSE"));
                JPanel active;
                JPanel passive;
                if (value == 1) {
                    active = trueAP.getPanel();
                    passive = falseAP.getPanel();
                } else {
                    active = falseAP.getPanel();
                    passive = trueAP.getPanel();
                }
                active.setBorder(BorderFactory.createLineBorder(Color.green));
                passive.setBorder(BorderFactory.createLineBorder(Color.red));
                Toolbox.setEnabledRecursive(active, true);
                Toolbox.setEnabledRecursive(passive, false);
            }
        });

        return new IfConditionPanel(result, combined);
    }

    @NotNull
    public static JPanel createLiveDocumentationPanel(UIContext uiContext) {
        JPanel liveDocs = new JPanel(new MigLayout(LAYOUT));

        liveDocs.add(createPanel(uiContext,"Fuel", FuelMathMeta.CONTENT), CONSTRAINTS);

        liveDocs.add(createPanel(uiContext, "tCharge", SpeedDensityMeta.CONTENT), CONSTRAINTS);

        liveDocs.add(createPanel(uiContext, "Idle", IdleThreadMeta.CONTENT), CONSTRAINTS);

        // todo: restore this functionality
        // liveDocs.add(createPanel("ETB", ElectronicThrottleMeta.CONTENT), CONSTRAINTS);

        return liveDocs;
    }

    public static JPanel createSensorsLiveDataPanel(UIContext uiContext) {
        JPanel liveDocs = new JPanel(new MigLayout(LAYOUT));

/*
        One day we shall have this back
        liveDocs.add(createPanel("Throttle Position Sensor", TpsMeta.TPS_SECTION), CONSTRAINTS);

 */

        liveDocs.add(createPanel(uiContext, "Trigger", TriggerDecoderMeta.CONTENT), CONSTRAINTS);

        return liveDocs;
    }
}
