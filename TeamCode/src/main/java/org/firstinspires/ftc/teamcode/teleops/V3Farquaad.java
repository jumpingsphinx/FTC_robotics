package org.firstinspires.ftc.teamcode.teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "V3Farquaad", group = "TeleOp")
public class V3Farquaad extends LinearOpMode {
    // DRIVE CONSTANTS
    public static final double DRIVER_SPEED_SCALAR = 0.85;
    public static final double DRIVER_SPRINT_MODE_SCALAR = 0.95;
    public static final double DRIVER_ROTATION_SCALAR = 0.7;
    public static final double DRIVER_SLOW_MODE_SCALAR = 0.50;
    public static final double SENSITIVITY_THRESHOLD = 0.20;
    public static final double LIFT_SCALAR = 0.85;
    public static final double PULL_SCALAR = 0.90;
    public static final double COURSE_CORRECT = 1.04;

    //GUNNER CONSTANTS
    public static final double HOPPER_OPEN = 0.05;
    public static final double HOPPER_CLOSED = 0.38;
    public static final double CLAW_CLOSED = 0.67;
    public static final double CLAW_OPEN_PICKUP = 0.52;
    public static final double CLAW_OPEN_DROPOFF = 0.58;
    public static final double WRIST_UP = 0.13;
    public static final double WRIST_SAFE = 0.6;
    public static final double WRIST_DOWN = 0.825;
    public static final double LAUNCHER_HOLD = 0.83;
    public static final double LAUNCHER_RELEASE = 0.68;
    public enum CLAW_STATE {
            DOWN_OPEN,
            DOWN_CLOSED,
            UP_OPEN,
            UP_CLOSED

    }
//
//    public double currentDistance = 0;

    private Servo wristleft;
    private Servo wristright;
    private Servo claw;
    private Servo hopper;
    private Servo launcher;

    //drive motors
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor lift;
    private DcMotor pullupleft;
    private DcMotor pullupright;
//    private DistanceSensor backDistanceSensor;
    private IMU imu;

    public enum SEQUENCE_STEP {
        INIT,
        CLOSE_CLAW,
        FLIP_WRIST_UP,
        CLOSE_HOPPER,
        OPEN_CLAW_FOR_DROPOFF,
        COMPLETE
    }

    RevBlinkinLedDriver blinkinLedDriver;
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        lift = hardwareMap.get(DcMotor.class, "lift");
        pullupleft = hardwareMap.get(DcMotor.class, "pullupleft");
        pullupright = hardwareMap.get(DcMotor.class, "pullupright");
        imu = hardwareMap.get(IMU.class, "imu");

        wristleft = hardwareMap.get(Servo.class, "wristleft");
        wristright = hardwareMap.get(Servo.class, "wristright");
        claw = hardwareMap.get(Servo.class, "claw");
        hopper = hardwareMap.get(Servo.class, "hopper");
        launcher = hardwareMap.get(Servo.class, "launcher");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

//        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backdistance");

        //tighten the launcher
        launcher.setPosition(LAUNCHER_HOLD);
        hopper.setPosition(HOPPER_CLOSED);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pullupleft.setDirection(DcMotor.Direction.FORWARD);
        pullupright.setDirection(DcMotorSimple.Direction.REVERSE);
        pullupleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullupright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TeleOpMecanumDrive myDrive = new TeleOpMecanumDrive(fl, fr, bl, br);
        double yaw = 0;

        ElapsedTime mRunTime = new ElapsedTime();
        boolean checkCase = false;
        boolean wasAPressed = false;
        boolean wasYPressed = false;
        boolean wasXPressed = false;
        boolean wasBPressed = false;
        boolean wasBumperTriggered = false;
        wristleft.setPosition(WRIST_DOWN);
        wristright.setPosition(1-WRIST_DOWN);
        claw.setPosition(CLAW_CLOSED);

        waitForStart();
        while (opModeIsActive()) {
            //gunner controls
            //claw logic
            if (gamepad2.y && !wasYPressed){
                wasYPressed = true;
                if (claw.getPosition() > CLAW_OPEN_PICKUP - 0.035 && claw.getPosition() < CLAW_OPEN_DROPOFF + 0.035){
                    claw.setPosition(CLAW_CLOSED);
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                }
                else if (claw.getPosition() > CLAW_CLOSED - 0.035 && claw.getPosition() < CLAW_CLOSED + 0.035){
                    if (wristleft.getPosition() > WRIST_DOWN - 0.035 && wristleft.getPosition() < WRIST_DOWN + 0.035){
                        claw.setPosition(CLAW_OPEN_PICKUP);
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    }
                    else if (wristleft.getPosition() > WRIST_UP - 0.035 && wristleft.getPosition() < WRIST_UP + 0.035){
                        claw.setPosition(CLAW_OPEN_DROPOFF);
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    }
                    else {
                        claw.setPosition(CLAW_OPEN_DROPOFF);
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    }
                }
            }
            else if (!gamepad2.y){
                wasYPressed = false;
            }
            if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                launcher.setPosition(LAUNCHER_RELEASE);
            }
            //wrist logic
            if (gamepad2.a && !wasAPressed){
                wasAPressed = true;
                if (wristleft.getPosition() > WRIST_DOWN - 0.035 && wristleft.getPosition() < WRIST_DOWN + 0.035){
                    wristleft.setPosition(WRIST_UP);
                    wristright.setPosition(1-WRIST_UP);
                }
                else if (wristleft.getPosition() > WRIST_UP - 0.035 && wristleft.getPosition() < WRIST_UP + 0.035) {
                    wristleft.setPosition(WRIST_DOWN);
                    wristright.setPosition(1 - WRIST_DOWN);
                }
                else if (wristleft.getPosition() > WRIST_SAFE - 0.035 && wristleft.getPosition() < WRIST_SAFE + 0.035) {
                    wristleft.setPosition(WRIST_DOWN);
                    wristright.setPosition(1 - WRIST_DOWN);
                }
            }
            else if (!gamepad2.a){
                wasAPressed = false;
            }


            // add 2 second wait and then auto-close
            if (gamepad2.left_bumper && !wasBumperTriggered){
                wasBumperTriggered = true;
                if (hopper.getPosition() > HOPPER_CLOSED - 0.035 && hopper.getPosition() < HOPPER_CLOSED + 0.035){
                    hopper.setPosition(HOPPER_OPEN);
                    mRunTime.reset();
                }
                else if (hopper.getPosition() > HOPPER_OPEN - 0.035 && hopper.getPosition() < HOPPER_OPEN + 0.035){
                    hopper.setPosition(HOPPER_CLOSED);
                }
            }
            else if (!gamepad2.left_bumper){
                wasBumperTriggered = false;
                if (mRunTime.time(TimeUnit.MILLISECONDS) > 2000){
                    hopper.setPosition(HOPPER_CLOSED);
                    mRunTime.reset();
                }
            }


            // lift controls
            if (Math.abs(gamepad2.left_stick_y) > SENSITIVITY_THRESHOLD){
                double power = -gamepad2.left_stick_y;
                lift.setPower(power * LIFT_SCALAR);
                if (power > 0) {
                    wristleft.setPosition(WRIST_SAFE);
                    wristright.setPosition(1 - WRIST_SAFE);
                }
            }
            else{
                lift.setPower(0);
            }
//
//            currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
//            if (gamepad1.x && !checkCase && !wasXPressed){
//                firstTime = System.currentTimeMillis();
//                if ((firstTime - recentTime ) < 250){
//                }
//                else if (currentDistance > 5.5){
//                    fl.setPower(-0.35);
//                    fr.setPower(-0.35);
//                    br.setPower(-0.35);
//                    bl.setPower(-0.35);
//                    checkCase = true;
//                    wasXPressed = true;
//                    continue;
//                }
//                else {
//                    gamepad1.rumble(20);
//                }
//            }
//            if (checkCase){
//                recentTime = System.currentTimeMillis();
//                if (currentDistance > 5.5){
//                    fl.setPower(-0.35);
//                    fr.setPower(-0.35);
//                    br.setPower(-0.35);
//                    bl.setPower(-0.35);
//                    wasXPressed = false;
//                    if (gamepad1.x && (recentTime - firstTime) > 125){
//                        checkCase = false;
//                    }
//                    continue;
//                }
//                else {
//                    wasXPressed = false;
//                    checkCase = false;
//                    gamepad1.rumble(20);
//                }
//            }


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            yaw = gamepad1.right_stick_x;

            if (Math.abs(yaw) < SENSITIVITY_THRESHOLD) {
                yaw = 0;
            }

            if (Math.abs(x) < SENSITIVITY_THRESHOLD){
                x = 0;
            }
            if (Math.abs(y) < SENSITIVITY_THRESHOLD) {
                y = 0;
            }
            double scalar = DRIVER_SPEED_SCALAR;
            if (gamepad1.a){
                scalar = DRIVER_SLOW_MODE_SCALAR;
            }
            //button b activates sprint mode
            else if (gamepad1.b){
                scalar = DRIVER_SPRINT_MODE_SCALAR;
            }

            telemetry.addData("Wheel powers, fl, fr, bl, br: ", myDrive.drive(x, y, yaw * DRIVER_ROTATION_SCALAR, scalar, COURSE_CORRECT));
            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.update();

            // pull up controls
            if (Math.abs(gamepad2.right_stick_y) > SENSITIVITY_THRESHOLD){
                double power = gamepad2.right_stick_y;
                pullupright.setPower(power * PULL_SCALAR);
                pullupleft.setPower(power * PULL_SCALAR);
            }
            else {
                pullupright.setPower(0);
                pullupleft.setPower(0);
            }
        }

    }

}

