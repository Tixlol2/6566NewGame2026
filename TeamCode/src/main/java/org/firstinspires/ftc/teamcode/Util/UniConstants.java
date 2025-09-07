package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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



}
