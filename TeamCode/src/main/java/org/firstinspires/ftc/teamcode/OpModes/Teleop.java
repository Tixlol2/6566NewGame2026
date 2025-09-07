package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDrive;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;


//Written by Noah Nottingham - 6566 Circuit Breakers
@Mercurial.Attach
@MecDrive.Attach
@TeleOp(name = "TeleOp", group = "Driver") //The name and group
//@Disabled //How you would disable/enable an opmode from appearing on the DS
public class Teleop extends OpMode {


    BoundGamepad driverGamepad;
    BoundGamepad gp2;



    UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    @Override
    public void init() {



        driverGamepad = Mercurial.gamepad1(); //Has drive
        gp2 = Mercurial.gamepad2();







        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void start(){



    }

    @Override
    public void loop() {





        telemetry.update();





    }
}
