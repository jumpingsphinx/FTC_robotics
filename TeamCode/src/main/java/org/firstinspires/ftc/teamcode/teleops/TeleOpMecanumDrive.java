package org.firstinspires.ftc.teamcode.teleops;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class TeleOpMecanumDrive{
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public TeleOpMecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public ArrayList drive(double x, double y, double yaw, double scalar, double correction) {
        double fl = x + y + yaw;
        double fr = -x + y - yaw;
        double bl = -x + y + yaw;
        double br = x + y - yaw;

        // Find the maximum absolute power that needs to be applied to any motor
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));

        // Scale the powers so that the maximum power is set to 1
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        if (scalar != 1.0){
            fl = fl * scalar;
            fr = fr * scalar;
            bl = bl * scalar;
            br = br * scalar;
        }
        if (correction != 1.0){
            fl = fl * correction;
        }

        // Set the power for the motors
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        ArrayList telemetrystuff = new ArrayList();

        telemetrystuff.add(frontLeft.getPower());
        telemetrystuff.add(frontRight.getPower());
        telemetrystuff.add(backLeft.getPower());
        telemetrystuff.add(backRight.getPower());

        return telemetrystuff;
    }
}