package org.firstinspires.ftc.teamcode.autos;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Wrapper {
    public static final double scalar = 0.7;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    public Wrapper(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }
    public ArrayList driveToEncoderPosition(int flticks, int blticks, int frticks,  int brticks){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setTargetPosition(flticks);
        backLeft.setTargetPosition(blticks);
        frontRight.setTargetPosition(frticks);
        backRight.setTargetPosition(brticks);

        double max = Math.max(Math.abs(flticks), Math.max(Math.abs(frticks), Math.max(Math.abs(blticks), Math.abs(brticks))));
        double flticksscaled = flticks / max;
        double frticksscaled = frticks / max;
        double blticksscaled = blticks / max;
        double brticksscaled = brticks / max;


        if (scalar != 1.0){
            flticksscaled *= scalar;
            frticksscaled *= scalar;
            blticksscaled *= scalar;
            brticksscaled *= scalar;
        }

        frontLeft.setPower(flticksscaled);
        backLeft.setPower(blticksscaled);
        frontRight.setPower(frticksscaled);
        backRight.setPower(brticksscaled);

        ArrayList telemetrystuff = new ArrayList();

        telemetrystuff.add(frontLeft.getCurrentPosition());
        telemetrystuff.add(frontRight.getCurrentPosition());
        telemetrystuff.add(backLeft.getCurrentPosition());
        telemetrystuff.add(backRight.getCurrentPosition());

        return telemetrystuff;
    }
}
