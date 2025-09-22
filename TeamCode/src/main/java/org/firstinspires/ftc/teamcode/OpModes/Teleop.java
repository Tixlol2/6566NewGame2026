package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;

import dev.frozenmilk.mercurial.Mercurial;


//Written by Noah Nottingham - 6566 Circuit Breakers
@MecDrive.Attach
@Mercurial.Attach
@TeleOp(name = "TeleOp", group = "Driver") //The name and group
//@Disabled //How you would disable/enable an opmode from appearing on the DS
public class Teleop extends OpMode {



    Follower follower;

    DcMotorEx active;
    DcMotorEx launcher;
    DcMotorEx rotary;

    int rotaryCurrentPosition = 0;


    ArrayList<UniConstants.slotState> slots = new ArrayList<>(List.of(UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY));
    ArrayList<ColorSensor> colorSensors = new ArrayList<>();


    //Launch velo in radians per second
    public static double launchVelocityRPS = 0;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        active = hardwareMap.get(DcMotorEx.class, UniConstants.ACTIVE_INTAKE_STRING);
        active.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcher = hardwareMap.get(DcMotorEx.class, UniConstants.LAUNCHER_STRING);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotary = hardwareMap.get(DcMotorEx.class, UniConstants.ROTARY_STRING);

        rotary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotary.setDirection(UniConstants.ROTARY_DIRECTION);

        colorSensors.addAll(
                List.of(
                    (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_1_STRING)),
                    (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_2_STRING)),
                    (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_3_STRING))
                )
        );




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void start(){
        for(ColorSensor sensor : colorSensors){
            sensor.enableLed(true);
        }


    }

    @Override
    public void loop() {



        if(gamepad1.a){
            active.setPower(.5);
            rotary.setPower(.5); //? Should rotary be spun at the same speed* as the active?
        }

        if(gamepad1.y){
            readSlots();
        }

        //If blue, aim for blue goal else aim for red goal - this target position will be fed to servo
        //Is it possible to send data from auto to tele? like whether blue or not
        //Think about only doing this every 500ms or so so we arent constantly computing
        //Tradeoff of accuracy vs chub efficiency

        //Velocity should be constantly interpolated via apriltag localization data right? Heavy comp power
        //This should be done in terms of radians per second somehow?? Like transcribe exit velo to radians per second for motor
        //TODO: Generate linear regression to determine velocity for given positions
        //Like, the radial velocity will be a function of the target exit velocity or smth
        launcher.setVelocity(launchVelocityRPS, AngleUnit.RADIANS);



        //Slot numbers are already known, 0 is ready to transfer into launcher, 1 is to the right, 2 is the last one.
        //So for obelisk PPG, 0 = P, 1 = P, 2 = G
        //If that is good, bool readyToLaunch = true
        //I need the robot to be built or cad to be done to actually do this without theory coding but shrug
        rotaryCurrentPosition = rotary.getCurrentPosition();

        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
        telemetry.addData("Slot 0 State ", slots.get(0));
        telemetry.addData("Slot 1 State ", slots.get(1));
        telemetry.addData("Slot 2 State ", slots.get(2));
        telemetry.addLine();
        telemetry.addData("Launch Velocity (Rads/s) ", launchVelocityRPS);
        telemetry.addData("Calculated Launch Velocity ", 0); //Todo: make work

        telemetry.update();





    }

    public void readSlots(){
        for(int i = 0; i < 3; i++){
            double hue = colorSensors.get(i).argb();
            if((UniConstants.PURPLE_ARTIFACT_LOWER_HUE < hue) && (hue < UniConstants.PURPLE_ARTIFACT_UPPER_HUE)){
                slots.set(i, UniConstants.slotState.PURPLE);
            } else if((UniConstants.GREEN_ARTIFACT_LOWER_HUE < hue) && (hue < UniConstants.GREEN_ARTIFACT_UPPER_HUE)){
                slots.set(i, UniConstants.slotState.GREEN);
            } else {
                slots.set(i, UniConstants.slotState.EMPTY);
            }
        }

    }

}
