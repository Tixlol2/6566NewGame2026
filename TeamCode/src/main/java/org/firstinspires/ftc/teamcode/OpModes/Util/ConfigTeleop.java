package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config //Allows ALL PUBLIC STATIC VARIABLES to be monitored on FTC Dash.
@TeleOp(name = "Config Teleop", group = "Example") //The name and group
public class ConfigTeleop extends OpMode {

    DcMotorEx motor;
    public static String name = "motor";

    public static double power = 0;



    @Override
    public void init() {
        //ONE TIME INIT CALL
        //This is where you will call all of the constructors for your different subsystems.
        //Eventually, once I have them setup, you will see examples like intake, outtake, etc.
        //Telemetry is also reinitialized here as to allow you to view it on FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        motor = hardwareMap.get(DcMotorEx.class, name);

    }




    @Override
    public void init_loop() {
        //INIT LOOP
        //We never put anything here for motors/servos
        //as its against the rules for the bots to move AT ALL
        //in the initialization phase of the Driver Controlled Period.
        //For testing purposes, obviously yes you can put stuff here.


        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start(){
        //ONE TIME DRIVER PERIOD CALL
        //something like starting positions :)
    }

    @Override
    public void loop() {
        //DRIVER LOOP
        //We will only ever use the LOGITECH CONTROLLER SCHEME
        //https://gm0.org/en/latest/docs/software/tutorials/gamepad.html




        //Gamepad 1
        if(gamepad1.a){
            telemetry.addLine("G1 BUTTON A HELD DOWN");
            motor.setPower(power);

        }
        if(gamepad1.b){
            telemetry.addLine("G1 BUTTON B HELD DOWN");
            motor.setPower(-power);
        }
        if(gamepad1.x){
            telemetry.addLine("G1 BUTTON X HELD DOWN");
            motor.setPower(0);
        }
        if(gamepad1.y){
            telemetry.addLine("G1 BUTTON Y HELD DOWN");

        }

        if(gamepad1.dpad_down){
            telemetry.addLine("G1 BUTTON DPAD DOWN HELD DOWN");
            power -= .1;
            power = Math.max(0, power);
        }
        if(gamepad1.dpad_right){
            telemetry.addLine("G1 BUTTON DPAD RIGHT HELD DOWN");

        }
        if(gamepad1.dpad_left){
            telemetry.addLine("G1 BUTTON DPAD LEFT HELD DOWN");

        }
        if(gamepad1.dpad_up){
            telemetry.addLine("G1 BUTTON DPAD UP HELD DOWN");
            power += .1;
            power = Math.min(1, power);
        }






        //Gamepad 2
        if(gamepad2.a){
            telemetry.addLine("G2 BUTTON A HELD DOWN");
        }
        if(gamepad2.b){
            telemetry.addLine("G2 BUTTON B HELD DOWN");

        }
        if(gamepad2.x){
            telemetry.addLine("G2 BUTTON X HELD DOWN");

        }
        if(gamepad2.y){
            telemetry.addLine("G2 BUTTON Y HELD DOWN");

        }

        if(gamepad2.dpad_down){
            telemetry.addLine("G2 BUTTON DPAD DOWN HELD DOWN");
        }
        if(gamepad2.dpad_right){
            telemetry.addLine("G2 BUTTON DPAD RIGHT HELD DOWN");

        }
        if(gamepad2.dpad_left){
            telemetry.addLine("G2 BUTTON DPAD LEFT HELD DOWN");

        }
        if(gamepad2.dpad_up){
            telemetry.addLine("G2 BUTTON DPAD UP HELD DOWN");

        }











        //Sends the 'packet' of telemetry to both the Driver Hub and FTCDashboard
        telemetry.addData("Power ", power);
        telemetry.addData("Velocity ", motor.getVelocity());
        telemetry.update();





    }
}
