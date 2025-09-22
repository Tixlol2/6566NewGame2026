package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;




@Config //Allows ALL PUBLIC STATIC VARIABLES to be monitored on FTC Dash.
@TeleOp(name = "Example Teleop", group = "Example") //The name and group
@Disabled //How you would disable/enable an opmode from appearing on the DS
public class ExampleTeleop extends OpMode {

    //Pedro Pathing Follower
    Follower follower;


    //Getting rid of magic numbers/variables
    double forwardDrive;
    double strafe;
    double heading;

    //For definitions of bot centric/field centric look HERE: https://docs.ftclib.org/ftclib/features/drivebases
    //Scroll down till it says "Control Scheme"
    //This can be helpful if you forget which is which
    public static boolean robotCentric = false;



    @Override
    public void init() {
        //ONE TIME INIT CALL
        //This is where you will call all of the constructors for your different subsystems.
        //Eventually, once I have them setup, you will see examples like intake, outtake, etc.
        //Telemetry is also reinitialized here as to allow you to view it on FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        follower = Constants.createFollower(hardwareMap);

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
        }
        if(gamepad1.b){
            telemetry.addLine("G1 BUTTON B HELD DOWN");

        }
        if(gamepad1.x){
            telemetry.addLine("G1 BUTTON X HELD DOWN");

        }
        if(gamepad1.y){
            telemetry.addLine("G1 BUTTON Y HELD DOWN");

        }

        if(gamepad1.dpad_down){
            telemetry.addLine("G1 BUTTON DPAD DOWN HELD DOWN");
        }
        if(gamepad1.dpad_right){
            telemetry.addLine("G1 BUTTON DPAD RIGHT HELD DOWN");

        }
        if(gamepad1.dpad_left){
            telemetry.addLine("G1 BUTTON DPAD LEFT HELD DOWN");

        }
        if(gamepad1.dpad_up){
            telemetry.addLine("G1 BUTTON DPAD UP HELD DOWN");

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







        //The reason we set these values here instead of using the Gamepad directly is because
        //We want to ELIMINATE magic numbers. https://en.wikipedia.org/wiki/Magic_number_(programming)
        //This reduces unreadability, and makes debugging easier on us in case we need to fix something.

        forwardDrive = -gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        heading = -gamepad1.right_stick_x;


        //Using the gamepad1 sticks, we set the different VECTORS that the drive will try to follow.
        follower.setTeleOpDrive(forwardDrive, strafe, heading, robotCentric);
        //Updates all aspects of the follower, including but not limited to its
        //Current pose data, power vectors, target poses, etc.
        follower.update();

        //Adding telemetry so that way you can monitor things that happen in a OpMode.
        //Telemetry is especially useful for things like motors so you can actively view the encoder positions.
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        //Sends the 'packet' of telemetry to both the Driver Hub and FTCDashboard
        telemetry.update();





    }
}
