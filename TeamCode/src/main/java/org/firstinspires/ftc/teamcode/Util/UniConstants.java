package org.firstinspires.ftc.teamcode.Util;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;

@Configurable
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

    public enum teamColor {
        RED,
        BLUE
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


    public static final double ANGLE_OF_LAUNCHER_IN_DEGREES = 35;
    public static  final double HEIGHT_OF_ROBOT_IN_METERS = 0.35;
    public static  final double HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS = (1.11125) - (HEIGHT_OF_ROBOT_IN_METERS);
    public static  final double MOTOR_TO_TURRET_RATIO = (double) 24 /155; //Motor to Turret
    public static  final double TURRET_TICKS_PER_DEGREE = 537.7/360;






}
