package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FatTonyDeconstructed extends OpMode {

    DcMotorEx motor;



    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
    }
}




