package com.opensr5.ini;

import com.opensr5.ini.field.*;
import org.jetbrains.annotations.Nullable;

import java.io.*;
import java.util.*;

/**
 * Andrey Belomutskiy, (c) 2013-2020
 * 12/23/2015.
 */
public class IniFileModel {
    public static final String RUSEFI_INI_PREFIX = "rusefi";
    public static final String RUSEFI_INI_SUFFIX = ".ini";
    public static final String INI_FILE_PATH = System.getProperty("ini_file_path", "..");
    private static final String SECTION_PAGE = "page";
    private static final String FIELD_TYPE_SCALAR = "scalar";
    private static final String FIELD_TYPE_STRING = "string";
    private static final String FIELD_TYPE_ARRAY = "array";
    private static final String FIELD_TYPE_BITS = "bits";

    private static IniFileModel INSTANCE;
    private String dialogId;
    private String dialogUiName;
    private Map<String, DialogModel> dialogs = new TreeMap<>();
    private Map<String, DialogModel.Field> allFields = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
    // this is only used while reading model - TODO extract reader
    private List<DialogModel.Field> fieldsOfCurrentDialog = new ArrayList<>();
    public Map<String, IniField> allIniFields = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);

    public Map<String, String> tooltips = new TreeMap<>();

    public static void main(String[] args) {
        System.out.println(IniFileModel.getInstance().dialogs);
    }

    private boolean isInSettingContextHelp = false;
    private boolean isInsidePageDefinition;

    public void readIniFile(String iniFilePath) {
        String fileName = findMetaInfoFile(iniFilePath);
        File input = null;
        if (fileName != null)
            input = new File(fileName);
        if (fileName == null || !input.exists()) {
            System.out.println("No such file: " + RUSEFI_INI_PREFIX + "*" + RUSEFI_INI_SUFFIX);
            return;
        }

        System.out.println("Reading " + fileName);
        RawIniFile content = IniFileReader.read(input);

        readIniFile(content);
    }

    public IniFileModel readIniFile(RawIniFile content) {
        for (RawIniFile.Line line : content.getLines()) {
            handleLine(line);
        }
        finishDialog();
        return this;
    }

    private static String findMetaInfoFile(String iniFilePath) {
        return findFile(iniFilePath, RUSEFI_INI_PREFIX, RUSEFI_INI_SUFFIX);
    }

    @Nullable
    public static String findFile(String fileDirectory, String prefix, String suffix) {
        File dir = new File(fileDirectory);
        if (!dir.isDirectory())
            return null;
        for (String file : dir.list()) {
            if (file.startsWith(prefix) && file.endsWith(suffix))
                return fileDirectory + File.separator + file;
        }
        return null;
    }

    private void finishDialog() {
        if (fieldsOfCurrentDialog.isEmpty())
            return;
        if (dialogUiName == null)
            dialogUiName = dialogId;
        dialogs.put(dialogUiName, new DialogModel(dialogId, dialogUiName, fieldsOfCurrentDialog));

        dialogId = null;
        fieldsOfCurrentDialog.clear();
    }

    private void handleLine(RawIniFile.Line line) {

        String rawTest = line.getRawText();
        try {
            LinkedList<String> list = new LinkedList<>(Arrays.asList(line.getTokens()));

            if (!list.isEmpty() && list.get(0).equals(SECTION_PAGE)) {
                isInsidePageDefinition = true;
                return;
            }

            // todo: use TSProjectConsumer constant
            if (isInSettingContextHelp) {
                // todo: use TSProjectConsumer constant
                if (rawTest.contains("SettingContextHelpEnd")) {
                    isInSettingContextHelp = false;
                }
                if (list.size() == 2)
                    tooltips.put(list.get(0), list.get(1));
                return;
            } else if (rawTest.contains("SettingContextHelp")) {
                isInsidePageDefinition = false;
                isInSettingContextHelp = true;
                return;
            }

            if (RawIniFile.Line.isCommentLine(rawTest))
                return;

            trim(list);

            if (list.isEmpty())
                return;

            if (isInsidePageDefinition) {
                handleFieldDefinition(list);
                return;
            }

            String first = list.getFirst();

            if ("dialog".equals(first)) {
                handleDialog(list);
            } else if ("field".equals(first)) {
                handleField(list);
            }
        } catch (RuntimeException e) {
            throw new IllegalStateException("While [" + rawTest + "]", e);
        }
    }

    private void handleFieldDefinition(LinkedList<String> list) {
        if (list.get(1).equals(FIELD_TYPE_SCALAR)) {
            registerField(ScalarIniField.parse(list));
        } else if (list.get(1).equals(FIELD_TYPE_STRING)) {
            registerField(StringIniField.parse(list));
        } else if (list.get(1).equals(FIELD_TYPE_ARRAY)) {
            registerField(ArrayIniField.parse(list));
        } else if (list.get(1).equals(FIELD_TYPE_BITS)) {
            registerField(EnumIniField.parse(list));
        } else {
            throw new IllegalStateException("Unexpected " + list);
        }
    }

    private void registerField(IniField field) {
        allIniFields.put(field.getName(), field);
    }

    private void handleField(LinkedList<String> list) {
        list.removeFirst(); // "field"

        String uiFieldName = list.isEmpty() ? "" : list.removeFirst();

        String key = list.isEmpty() ? null : list.removeFirst();

        DialogModel.Field field = new DialogModel.Field(key, uiFieldName);
        if (key != null) {
            // UI labels do not have 'key'
            allFields.put(key, field);
        }
        fieldsOfCurrentDialog.add(field);
        System.out.println("IniFileModel: Field label=[" + uiFieldName + "] : key=[" + key + "]");
    }

    public Map<String, DialogModel.Field> getAllFields() {
        return allFields;
    }

    @Nullable
    public DialogModel.Field getField(String key) {
        return allFields.get(key);
    }

    private void handleDialog(LinkedList<String> list) {
        finishDialog();
        list.removeFirst(); // "dialog"
        //                    trim(list);
        String keyword = list.removeFirst();
//                    trim(list);
        String name = list.isEmpty() ? null : list.removeFirst();

        dialogId = keyword;
        dialogUiName = name;
        System.out.println("IniFileModel: Dialog key=" + keyword + ": name=[" + name + "]");
    }

    private void trim(LinkedList<String> list) {
        while (!list.isEmpty() && list.getFirst().isEmpty())
            list.removeFirst();
    }

    enum State {
        SKIPPING,
        DIALOG
    }

    public static synchronized IniFileModel getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IniFileModel();
            INSTANCE.readIniFile(INI_FILE_PATH);
        }
        return INSTANCE;
    }

    public Map<String, DialogModel> getDialogs() {
        return dialogs;
    }
}
