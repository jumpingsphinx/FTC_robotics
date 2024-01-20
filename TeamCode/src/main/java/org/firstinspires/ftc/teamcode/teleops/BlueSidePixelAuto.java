package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RedVisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.vision.BlueVisionPipeline;

@Config
@Autonomous(name = "BLUE_AUTO_PIXEL", group = "Autonomous")
public class BlueSidePixelAuto extends LinearOpMode {
    boolean finished = false;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Servo wristleft = hardwareMap.get(Servo.class, "wristleft");
        Servo wristright = hardwareMap.get(Servo.class, "wristright");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotor lift = hardwareMap.get(DcMotor.class,  "lift");


        //vision here// or 2 or 3

        Action TrajectoryAction1;
        Action TrajectoryAction2;
        Action TrajectoryAction3;
        TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                // drop pixel here
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                // drop pixel here
                .waitSeconds(3)
                .build();
        TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                // DROP PIXEL
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46,Math.toRadians(180))
                // DROP OTHER PIXEL
                .waitSeconds(3)
                .build();
        TrajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                // drop pixel here
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                // drop pixel here
                .waitSeconds(3)
                .build();
        //robot moves on init!!
        claw.setPosition(0.55);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueVisionPipeline pipeline = new BlueVisionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "failed to open camera");
                telemetry.update();
            }
        });

        while(!isStopRequested() && !opModeIsActive()) {
            int position = pipeline.outputPosition();
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = pipeline.outputPosition();
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action TrajectoryActionChosen;
        if (startPosition == 1){
            TrajectoryActionChosen = TrajectoryAction1;
        }
        else if (startPosition == 2){
            TrajectoryActionChosen = TrajectoryAction2;
        }
        else {
            TrajectoryActionChosen = TrajectoryAction3;
        }

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryActionChosen, // Example of a drive action
                        (telemetryPacket) -> {
                            telemetry.addLine("Ran to position!");
                            return false;
                        },
                        (telemetryPacket) -> { // Run some action
                            wristleft.setPosition(0.03);
                            wristright.setPosition(0.97);
                            return false;
                        },
                        (telemetryPacket) -> {
                            claw.setPosition(0.32);
                            return false;
                        },
                        (telemetryPacket) -> {
                            wristleft.setPosition(0.68);
                            wristright.setPosition(.32);
                            return false;
                        },
                        (telemetryPacket) -> {
                            claw.setPosition(0.55);
                            return false;
                        }));

    }

}