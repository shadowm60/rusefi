package com.rusefi;

import com.rusefi.config.generated.Fields;
import com.rusefi.ui.UIContext;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionListener;

/**
 * Andrey Belomutskiy, (c) 2013-2020
 */
public class PresetsPane {
    // see rusefi_enums.h
    private static final int TEST_V_12 = 49;
    private static final int ETB_BENCH = 58;
    private static final int MINIMAL_PINS = 99;
    private final UIContext uiContext;

    private final JPanel content = new JPanel(new GridLayout(4, 5));

    public PresetsPane(UIContext uiContext) {
        this.uiContext = uiContext;
        content.add(new SetEngineTypeCommandControl(uiContext,"Frankenso Miata NA6 Stage 0", "/engines/miata_na.png", Fields.ET_FRANKENSO_MIATA_NA6_VAF).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Frankenso Miata NA6 Stage 1", "/engines/miata_na.png", Fields.ET_FRANKENSO_MIATA_NA6).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Frankenso Miata NB2", "/engines/miata_nb.png", Fields.ET_FRANKENSO_MIATA_NB2).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata NA6 MAP", "engines/miata_na.png", Fields.ET_MRE_MIATA_NA6_MAP).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata NA6 VAF", "engines/miata_na.png", Fields.ET_MRE_MIATA_NA6_VAF).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata 94 MAP", "engines/miata_na.png", Fields.ET_MRE_MIATA_94_MAP).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata NB2 MAP", "engines/miata_nb.png", Fields.ET_MRE_MIATA_NB2_MAP).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata NB2 MAF", "engines/miata_nb.png", Fields.ET_MRE_MIATA_NB2_MAF).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Proteus NB2 MAP", "engines/miata_nb.png", Fields.ET_PROTEUS_MIATA_NB2).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Hellen72 NB2 MAP", "engines/miata_nb.png", Fields.ET_HELLEN_NB2).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE Miata Defaults", "", Fields.ET_MRE_DEFAULTS).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Frankenso V12", "/engines/v12", TEST_V_12).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "ETB bench", "", ETB_BENCH).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "Minimal", "", MINIMAL_PINS).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE old test", "", Fields.ET_MRE_OLD_TEST_BOARD).getContent());
        content.add(new SetEngineTypeCommandControl(uiContext, "MRE new test", "", Fields.ET_MRE_NEW_TEST_BOARD).getContent());
    }

    public JPanel getContent() {
        return content;
    }

    private class SetEngineTypeCommandControl extends FixedCommandControl {
        private final String labelTest;

        public SetEngineTypeCommandControl(UIContext uiContext, String labelTest, String imageFileName, int engineType) {
            super(uiContext, labelTest, imageFileName, CommandControl.SET, "set " + Fields.CMD_ENGINE_TYPE + " " + engineType);
            this.labelTest = labelTest;
        }

        @NotNull
        @Override
        protected ActionListener createButtonListener() {
            return e -> {
                int dialogResult = JOptionPane.showConfirmDialog(panel, "Do you really want to reset all settings to " + labelTest,
                        "Warning", JOptionPane.YES_NO_OPTION);
                if (dialogResult != JOptionPane.YES_OPTION)
                    return;

                uiContext.getCommandQueue().write(getCommand());
            };
        }
    }
}
