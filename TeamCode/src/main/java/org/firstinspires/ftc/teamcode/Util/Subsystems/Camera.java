//package org.firstinspires.ftc.teamcode.Util.Subsystems;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Util.UniConstants;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//
//import java.lang.annotation.ElementType;
//import java.lang.annotation.Inherited;
//import java.lang.annotation.Retention;
//import java.lang.annotation.RetentionPolicy;
//import java.lang.annotation.Target;
//import java.util.ArrayList;
//import java.util.List;
//
//import dev.frozenmilk.dairy.core.dependency.Dependency;
//import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
//import dev.frozenmilk.dairy.core.wrapper.Wrapper;
//import dev.frozenmilk.mercurial.subsystems.Subsystem;
//import kotlin.annotation.MustBeDocumented;
//
//public class Camera implements Subsystem {
//
//    public static Telemetry telemetry;
//
//    AprilTagProcessor ATProcessor = UniConstants.aprilTagProcessor;
//    ArrayList<AprilTagDetection> detections = new ArrayList<>();
//    ArrayList<Integer> tagIDs = new ArrayList<>();
//    ColorBlobLocatorProcessor CLGProcessor = UniConstants.colorLocatorGreen;
//    ColorBlobLocatorProcessor CLPProcessor = UniConstants.colorLocatorPurple;
//
//
//    @Override
//    public void preUserInitHook(@NonNull Wrapper opMode) {
//
//        telemetry = opMode.getOpMode().telemetry;
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .addProcessor(ATProcessor)
//                .addProcessor(CLGProcessor)
//                .addProcessor(CLPProcessor)
//                .setCamera(opMode.getOpMode().hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//    }
//
//    @Override
//    public void preUserLoopHook(@NonNull Wrapper opMode){
//
//    }
//
//    private void getAprilTagDetections(){
//        detections = ATProcessor.getDetections();
//        for(AprilTagDetection detection : detections){
//            tagIDs.add(detection.id);
//        }
//    }
//
//    private void getSetObeliskID(){
//        for(int id : tagIDs){
//            if(List.of(21, 22, 23).contains(id)){
//                UniConstants.obeliskID = id;
//            }
//        }
//    }
//
//    private void getCorrectGoalPose(UniConstants.TeamColor color){
//
//        switch(color){
//            case BLUE:
//                if(tagIDs.contains(UniConstants.BLUE_GOAL_ID)){
//                    AprilTagDetection detection = detections.get(tagIDs.indexOf(UniConstants.BLUE_GOAL_ID));
//                    telemetry.addData("Goal Tag X ", detection.ftcPose.x);
//                    telemetry.addData("Goal Tag Y ", detection.ftcPose.y);
//                    telemetry.addData("Goal Tag Z ", detection.ftcPose.z);
//                    telemetry.update();
//                }
//                break;
//            case RED:
//
//                break;
//            default:
//                break;
//        }
//
//    }
//
//    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
//    @Inherited
//    public @interface Attach { }
//
//    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(MecDrive.Attach.class));
//
//    @NonNull
//    @Override
//    public Dependency<?> getDependency() { return dependency; }
//
//    @Override
//    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
//}
