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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.Servo;

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


        //vision here
        double vision_result = 1; // or 2 or 3

        Action TrajectoryAction1;
        if (vision_result == 1){
            TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToX(14)
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .lineToY(-5)
                .build();}
        else if (vision_result == 2){
            TrajectoryAction1 = drive.actionBuilder(drive.pose)
                    .lineToX(20)
                    .build();}
        else {
            TrajectoryAction1 = drive.actionBuilder(drive.pose)
                    .lineToX(14)
                    .splineTo(new Vector2d(0, 0), Math.toRadians(-90))
                    .lineToY(-5)
                    .build();}


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1, // Example of a drive action
                        (telemetryPacket) -> {
                            telemetry.addLine("Action!");
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