package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;

@Config
public class UniConstants {



    //Drive
    public static final String DRIVE_FRONT_LEFT_STRING = "LFM";
    public static final String DRIVE_FRONT_RIGHT_STRING = "RFM";
    public static final String DRIVE_BACK_LEFT_STRING = "LRM";
    public static final String DRIVE_BACK_RIGHT_STRING = "RRM";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;

    public enum loggingState{
        DISABLED,
        ENABLED,
        EXTREME
    }

    public enum slotState{
        PURPLE,
        GREEN,
        EMPTY
    }

    public static final String COLOR_SENSOR_SLOT_1_STRING = "COLOR0";
    public static final String COLOR_SENSOR_SLOT_2_STRING = "COLOR1";
    public static final String COLOR_SENSOR_SLOT_3_STRING = "COLOR2";

    public static final String ACTIVE_INTAKE_STRING = "ACTIVE";
    public static final String LAUNCHER_STRING  = "LAUNCHER";
    public static final String ROTARY_STRING = "ROTARY";
    public static final DcMotorSimple.Direction ROTARY_DIRECTION = DcMotorSimple.Direction.FORWARD; //TODO: Ensure this is correct direction


    public static int PURPLE_ARTIFACT_UPPER_HUE = 350;
    public static int PURPLE_ARTIFACT_LOWER_HUE = 275;

    public static int GREEN_ARTIFACT_UPPER_HUE = 150;
    public static int GREEN_ARTIFACT_LOWER_HUE = 100;

    public static final int SPACE_BETWEEN_ROTARY_SLOTS = 300;









}
