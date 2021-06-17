package com.rusefi.config.generated;

// this file was generated automatically by rusEFI tool ConfigDefinition.jar based on (unknown script) integration/engine_state.txt Sat May 08 21:43:54 UTC 2021

// by class com.rusefi.output.FileJavaFieldsConsumer
import com.rusefi.config.*;

public class EngineState {
	public static final Field ISTCHARGEAIRMODEL = Field.create("ISTCHARGEAIRMODEL", 0, FieldType.BIT, 0);
	public static final Field UNUSEDBIT_1_1 = Field.create("UNUSEDBIT_1_1", 0, FieldType.BIT, 1);
	public static final Field UNUSEDBIT_1_2 = Field.create("UNUSEDBIT_1_2", 0, FieldType.BIT, 2);
	public static final Field UNUSEDBIT_1_3 = Field.create("UNUSEDBIT_1_3", 0, FieldType.BIT, 3);
	public static final Field UNUSEDBIT_1_4 = Field.create("UNUSEDBIT_1_4", 0, FieldType.BIT, 4);
	public static final Field UNUSEDBIT_1_5 = Field.create("UNUSEDBIT_1_5", 0, FieldType.BIT, 5);
	public static final Field UNUSEDBIT_1_6 = Field.create("UNUSEDBIT_1_6", 0, FieldType.BIT, 6);
	public static final Field UNUSEDBIT_1_7 = Field.create("UNUSEDBIT_1_7", 0, FieldType.BIT, 7);
	public static final Field UNUSEDBIT_1_8 = Field.create("UNUSEDBIT_1_8", 0, FieldType.BIT, 8);
	public static final Field UNUSEDBIT_1_9 = Field.create("UNUSEDBIT_1_9", 0, FieldType.BIT, 9);
	public static final Field UNUSEDBIT_1_10 = Field.create("UNUSEDBIT_1_10", 0, FieldType.BIT, 10);
	public static final Field UNUSEDBIT_1_11 = Field.create("UNUSEDBIT_1_11", 0, FieldType.BIT, 11);
	public static final Field UNUSEDBIT_1_12 = Field.create("UNUSEDBIT_1_12", 0, FieldType.BIT, 12);
	public static final Field UNUSEDBIT_1_13 = Field.create("UNUSEDBIT_1_13", 0, FieldType.BIT, 13);
	public static final Field UNUSEDBIT_1_14 = Field.create("UNUSEDBIT_1_14", 0, FieldType.BIT, 14);
	public static final Field UNUSEDBIT_1_15 = Field.create("UNUSEDBIT_1_15", 0, FieldType.BIT, 15);
	public static final Field UNUSEDBIT_1_16 = Field.create("UNUSEDBIT_1_16", 0, FieldType.BIT, 16);
	public static final Field UNUSEDBIT_1_17 = Field.create("UNUSEDBIT_1_17", 0, FieldType.BIT, 17);
	public static final Field UNUSEDBIT_1_18 = Field.create("UNUSEDBIT_1_18", 0, FieldType.BIT, 18);
	public static final Field UNUSEDBIT_1_19 = Field.create("UNUSEDBIT_1_19", 0, FieldType.BIT, 19);
	public static final Field UNUSEDBIT_1_20 = Field.create("UNUSEDBIT_1_20", 0, FieldType.BIT, 20);
	public static final Field UNUSEDBIT_1_21 = Field.create("UNUSEDBIT_1_21", 0, FieldType.BIT, 21);
	public static final Field UNUSEDBIT_1_22 = Field.create("UNUSEDBIT_1_22", 0, FieldType.BIT, 22);
	public static final Field UNUSEDBIT_1_23 = Field.create("UNUSEDBIT_1_23", 0, FieldType.BIT, 23);
	public static final Field UNUSEDBIT_1_24 = Field.create("UNUSEDBIT_1_24", 0, FieldType.BIT, 24);
	public static final Field UNUSEDBIT_1_25 = Field.create("UNUSEDBIT_1_25", 0, FieldType.BIT, 25);
	public static final Field UNUSEDBIT_1_26 = Field.create("UNUSEDBIT_1_26", 0, FieldType.BIT, 26);
	public static final Field UNUSEDBIT_1_27 = Field.create("UNUSEDBIT_1_27", 0, FieldType.BIT, 27);
	public static final Field UNUSEDBIT_1_28 = Field.create("UNUSEDBIT_1_28", 0, FieldType.BIT, 28);
	public static final Field UNUSEDBIT_1_29 = Field.create("UNUSEDBIT_1_29", 0, FieldType.BIT, 29);
	public static final Field UNUSEDBIT_1_30 = Field.create("UNUSEDBIT_1_30", 0, FieldType.BIT, 30);
	public static final Field UNUSEDBIT_1_31 = Field.create("UNUSEDBIT_1_31", 0, FieldType.BIT, 31);
	public static final Field AIRMASSINONECYLINDER = Field.create("AIRMASSINONECYLINDER", 4, FieldType.FLOAT);
	public static final Field TCHARGE = Field.create("TCHARGE", 8, FieldType.FLOAT);
	public static final Field TCHARGEK = Field.create("TCHARGEK", 12, FieldType.FLOAT);
	public static final Field TCHARGE_COFF = Field.create("TCHARGE_COFF", 16, FieldType.FLOAT);
	public static final Field AIRFLOW = Field.create("AIRFLOW", 20, FieldType.FLOAT);
	public static final Field MANIFOLDAIRPRESSUREACCELERATIONADJUSTMENT = Field.create("MANIFOLDAIRPRESSUREACCELERATIONADJUSTMENT", 24, FieldType.FLOAT);
	public static final Field ADJUSTEDMANIFOLDAIRPRESSURE = Field.create("ADJUSTEDMANIFOLDAIRPRESSURE", 28, FieldType.FLOAT);
	public static final Field IDLESTATE = Field.create("IDLESTATE", 32, FieldType.INT);
	public static final Field CURRENTIDLEPOSITION = Field.create("CURRENTIDLEPOSITION", 36, FieldType.FLOAT);
	public static final Field BASEIDLEPOSITION = Field.create("BASEIDLEPOSITION", 40, FieldType.FLOAT);
	public static final Field THROTTLEPEDALUPSTATE = Field.create("THROTTLEPEDALUPSTATE", 44, FieldType.INT);
	public static final Field ETBIDLEADDITION = Field.create("ETBIDLEADDITION", 48, FieldType.FLOAT);
	public static final Field TARGETAFR = Field.create("TARGETAFR", 52, FieldType.FLOAT);
	public static final Field ENGINECYCLEDURATIONMS = Field.create("ENGINECYCLEDURATIONMS", 56, FieldType.FLOAT);
	public static final Field MINRPMKCURRENTTPS = Field.create("MINRPMKCURRENTTPS", 60, FieldType.FLOAT);
	public static final Field CURRENTTPSADC = Field.create("CURRENTTPSADC", 64, FieldType.INT);
	public static final Field TPSVOLTAGEMCU = Field.create("TPSVOLTAGEMCU", 68, FieldType.FLOAT);
	public static final Field TPSVOLTAGEBOARD = Field.create("TPSVOLTAGEBOARD", 72, FieldType.FLOAT);
	public static final Field CURRENTBAROCORRECTEDVE = Field.create("CURRENTBAROCORRECTEDVE", 76, FieldType.FLOAT);
	public static final Field INJECTORFLOWCORRECTION = Field.create("INJECTORFLOWCORRECTION", 80, FieldType.FLOAT);
	public static final Field BAROCORRECTION = Field.create("BAROCORRECTION", 84, FieldType.FLOAT);
	public static final Field ISCRANKINGSTATE = Field.create("ISCRANKINGSTATE", 88, FieldType.BIT, 0);
	public static final Field ISAUTOMATICIDLE = Field.create("ISAUTOMATICIDLE", 88, FieldType.BIT, 1);
	public static final Field UNUSEDBIT_13_2 = Field.create("UNUSEDBIT_13_2", 88, FieldType.BIT, 2);
	public static final Field UNUSEDBIT_13_3 = Field.create("UNUSEDBIT_13_3", 88, FieldType.BIT, 3);
	public static final Field UNUSEDBIT_13_4 = Field.create("UNUSEDBIT_13_4", 88, FieldType.BIT, 4);
	public static final Field UNUSEDBIT_13_5 = Field.create("UNUSEDBIT_13_5", 88, FieldType.BIT, 5);
	public static final Field UNUSEDBIT_13_6 = Field.create("UNUSEDBIT_13_6", 88, FieldType.BIT, 6);
	public static final Field UNUSEDBIT_13_7 = Field.create("UNUSEDBIT_13_7", 88, FieldType.BIT, 7);
	public static final Field UNUSEDBIT_13_8 = Field.create("UNUSEDBIT_13_8", 88, FieldType.BIT, 8);
	public static final Field UNUSEDBIT_13_9 = Field.create("UNUSEDBIT_13_9", 88, FieldType.BIT, 9);
	public static final Field UNUSEDBIT_13_10 = Field.create("UNUSEDBIT_13_10", 88, FieldType.BIT, 10);
	public static final Field UNUSEDBIT_13_11 = Field.create("UNUSEDBIT_13_11", 88, FieldType.BIT, 11);
	public static final Field UNUSEDBIT_13_12 = Field.create("UNUSEDBIT_13_12", 88, FieldType.BIT, 12);
	public static final Field UNUSEDBIT_13_13 = Field.create("UNUSEDBIT_13_13", 88, FieldType.BIT, 13);
	public static final Field UNUSEDBIT_13_14 = Field.create("UNUSEDBIT_13_14", 88, FieldType.BIT, 14);
	public static final Field UNUSEDBIT_13_15 = Field.create("UNUSEDBIT_13_15", 88, FieldType.BIT, 15);
	public static final Field UNUSEDBIT_13_16 = Field.create("UNUSEDBIT_13_16", 88, FieldType.BIT, 16);
	public static final Field UNUSEDBIT_13_17 = Field.create("UNUSEDBIT_13_17", 88, FieldType.BIT, 17);
	public static final Field UNUSEDBIT_13_18 = Field.create("UNUSEDBIT_13_18", 88, FieldType.BIT, 18);
	public static final Field UNUSEDBIT_13_19 = Field.create("UNUSEDBIT_13_19", 88, FieldType.BIT, 19);
	public static final Field UNUSEDBIT_13_20 = Field.create("UNUSEDBIT_13_20", 88, FieldType.BIT, 20);
	public static final Field UNUSEDBIT_13_21 = Field.create("UNUSEDBIT_13_21", 88, FieldType.BIT, 21);
	public static final Field UNUSEDBIT_13_22 = Field.create("UNUSEDBIT_13_22", 88, FieldType.BIT, 22);
	public static final Field UNUSEDBIT_13_23 = Field.create("UNUSEDBIT_13_23", 88, FieldType.BIT, 23);
	public static final Field UNUSEDBIT_13_24 = Field.create("UNUSEDBIT_13_24", 88, FieldType.BIT, 24);
	public static final Field UNUSEDBIT_13_25 = Field.create("UNUSEDBIT_13_25", 88, FieldType.BIT, 25);
	public static final Field UNUSEDBIT_13_26 = Field.create("UNUSEDBIT_13_26", 88, FieldType.BIT, 26);
	public static final Field UNUSEDBIT_13_27 = Field.create("UNUSEDBIT_13_27", 88, FieldType.BIT, 27);
	public static final Field UNUSEDBIT_13_28 = Field.create("UNUSEDBIT_13_28", 88, FieldType.BIT, 28);
	public static final Field UNUSEDBIT_13_29 = Field.create("UNUSEDBIT_13_29", 88, FieldType.BIT, 29);
	public static final Field UNUSEDBIT_13_30 = Field.create("UNUSEDBIT_13_30", 88, FieldType.BIT, 30);
	public static final Field UNUSEDBIT_13_31 = Field.create("UNUSEDBIT_13_31", 88, FieldType.BIT, 31);
	public static final Field CRANKING_BASEFUEL = Field.create("CRANKING_BASEFUEL", 92, FieldType.FLOAT);
	public static final Field CRANKING_COOLANTTEMPERATURECOEFFICIENT = Field.create("CRANKING_COOLANTTEMPERATURECOEFFICIENT", 96, FieldType.FLOAT);
	public static final Field CRANKING_TPSCOEFFICIENT = Field.create("CRANKING_TPSCOEFFICIENT", 100, FieldType.FLOAT);
	public static final Field CRANKING_DURATIONCOEFFICIENT = Field.create("CRANKING_DURATIONCOEFFICIENT", 104, FieldType.FLOAT);
	public static final Field CRANKING_FUEL = Field.create("CRANKING_FUEL", 108, FieldType.FLOAT);
	public static final Field RUNNING_POSTCRANKINGFUELCORRECTION = Field.create("RUNNING_POSTCRANKINGFUELCORRECTION", 112, FieldType.FLOAT);
	public static final Field RUNNING_INTAKETEMPERATURECOEFFICIENT = Field.create("RUNNING_INTAKETEMPERATURECOEFFICIENT", 116, FieldType.FLOAT);
	public static final Field RUNNING_COOLANTTEMPERATURECOEFFICIENT = Field.create("RUNNING_COOLANTTEMPERATURECOEFFICIENT", 120, FieldType.FLOAT);
	public static final Field RUNNING_TIMESINCECRANKINGINSECS = Field.create("RUNNING_TIMESINCECRANKINGINSECS", 124, FieldType.FLOAT);
	public static final Field RUNNING_INJECTORLAG = Field.create("RUNNING_INJECTORLAG", 128, FieldType.FLOAT);
	public static final Field RUNNING_BASEFUEL = Field.create("RUNNING_BASEFUEL", 132, FieldType.FLOAT);
	public static final Field RUNNING_FUEL = Field.create("RUNNING_FUEL", 136, FieldType.FLOAT);
	public static final Field ETBFEEDFORWARD = Field.create("ETBFEEDFORWARD", 140, FieldType.FLOAT);
	public static final Field TARGETFROMTABLE = Field.create("TARGETFROMTABLE", 144, FieldType.FLOAT);
	public static final Field SPARKDWELL = Field.create("SPARKDWELL", 148, FieldType.FLOAT);
	public static final Field DWELLANGLE = Field.create("DWELLANGLE", 152, FieldType.FLOAT);
	public static final Field CLTTIMINGCORRECTION = Field.create("CLTTIMINGCORRECTION", 156, FieldType.FLOAT);
	public static final Field[] VALUES = {
	ISTCHARGEAIRMODEL,
	UNUSEDBIT_1_1,
	UNUSEDBIT_1_2,
	UNUSEDBIT_1_3,
	UNUSEDBIT_1_4,
	UNUSEDBIT_1_5,
	UNUSEDBIT_1_6,
	UNUSEDBIT_1_7,
	UNUSEDBIT_1_8,
	UNUSEDBIT_1_9,
	UNUSEDBIT_1_10,
	UNUSEDBIT_1_11,
	UNUSEDBIT_1_12,
	UNUSEDBIT_1_13,
	UNUSEDBIT_1_14,
	UNUSEDBIT_1_15,
	UNUSEDBIT_1_16,
	UNUSEDBIT_1_17,
	UNUSEDBIT_1_18,
	UNUSEDBIT_1_19,
	UNUSEDBIT_1_20,
	UNUSEDBIT_1_21,
	UNUSEDBIT_1_22,
	UNUSEDBIT_1_23,
	UNUSEDBIT_1_24,
	UNUSEDBIT_1_25,
	UNUSEDBIT_1_26,
	UNUSEDBIT_1_27,
	UNUSEDBIT_1_28,
	UNUSEDBIT_1_29,
	UNUSEDBIT_1_30,
	UNUSEDBIT_1_31,
	AIRMASSINONECYLINDER,
	TCHARGE,
	TCHARGEK,
	TCHARGE_COFF,
	AIRFLOW,
	MANIFOLDAIRPRESSUREACCELERATIONADJUSTMENT,
	ADJUSTEDMANIFOLDAIRPRESSURE,
	IDLESTATE,
	CURRENTIDLEPOSITION,
	BASEIDLEPOSITION,
	THROTTLEPEDALUPSTATE,
	ETBIDLEADDITION,
	TARGETAFR,
	ENGINECYCLEDURATIONMS,
	MINRPMKCURRENTTPS,
	CURRENTTPSADC,
	TPSVOLTAGEMCU,
	TPSVOLTAGEBOARD,
	CURRENTBAROCORRECTEDVE,
	INJECTORFLOWCORRECTION,
	BAROCORRECTION,
	ISCRANKINGSTATE,
	ISAUTOMATICIDLE,
	UNUSEDBIT_13_2,
	UNUSEDBIT_13_3,
	UNUSEDBIT_13_4,
	UNUSEDBIT_13_5,
	UNUSEDBIT_13_6,
	UNUSEDBIT_13_7,
	UNUSEDBIT_13_8,
	UNUSEDBIT_13_9,
	UNUSEDBIT_13_10,
	UNUSEDBIT_13_11,
	UNUSEDBIT_13_12,
	UNUSEDBIT_13_13,
	UNUSEDBIT_13_14,
	UNUSEDBIT_13_15,
	UNUSEDBIT_13_16,
	UNUSEDBIT_13_17,
	UNUSEDBIT_13_18,
	UNUSEDBIT_13_19,
	UNUSEDBIT_13_20,
	UNUSEDBIT_13_21,
	UNUSEDBIT_13_22,
	UNUSEDBIT_13_23,
	UNUSEDBIT_13_24,
	UNUSEDBIT_13_25,
	UNUSEDBIT_13_26,
	UNUSEDBIT_13_27,
	UNUSEDBIT_13_28,
	UNUSEDBIT_13_29,
	UNUSEDBIT_13_30,
	UNUSEDBIT_13_31,
	CRANKING_BASEFUEL,
	CRANKING_COOLANTTEMPERATURECOEFFICIENT,
	CRANKING_TPSCOEFFICIENT,
	CRANKING_DURATIONCOEFFICIENT,
	CRANKING_FUEL,
	RUNNING_POSTCRANKINGFUELCORRECTION,
	RUNNING_INTAKETEMPERATURECOEFFICIENT,
	RUNNING_COOLANTTEMPERATURECOEFFICIENT,
	RUNNING_TIMESINCECRANKINGINSECS,
	RUNNING_INJECTORLAG,
	RUNNING_BASEFUEL,
	RUNNING_FUEL,
	ETBFEEDFORWARD,
	TARGETFROMTABLE,
	SPARKDWELL,
	DWELLANGLE,
	CLTTIMINGCORRECTION,
	};
}
