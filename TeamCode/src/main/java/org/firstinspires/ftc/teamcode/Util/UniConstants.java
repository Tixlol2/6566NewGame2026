package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    public final static HashMap<Integer, String> ID_TO_NAME = new HashMap<Integer, String>() {{
        put(20, "BLUE GOAL");
        put(21, "GPP");
        put(22, "PGP");
        put(23, "PPG");
        put(24, "RED GOAL");

    }};




}
