package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Disabled

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(-90)));

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(37)
                        .setTangent(Math.toRadians(0))
                        .lineToX(18)
                        .waitSeconds(3)
                        .lineToY(40)
                        .turnTo(Math.toRadians(180))
                        .setTangent(Math.toRadians(0))
                        .lineToX(48)
                        .setTangent(Math.toRadians(90))
                        .lineToY(35)
                        .waitSeconds(3)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        } else {
            throw new AssertionError();
        }
    }
}
