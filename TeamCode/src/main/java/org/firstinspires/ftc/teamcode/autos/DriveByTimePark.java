package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "DRIVE BY TIME PARK", group = "Autonomous")
public class DriveByTimePark extends LinearOpMode {
    boolean finished = false;

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor fr = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor bl = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor br = hardwareMap.get(DcMotor.class, "rightBack");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;
        fl.setPower(-1);
        fr.setPower(-1);
        bl.setPower(-1);
        br.setPower(-1);
        sleep(2000);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

}}