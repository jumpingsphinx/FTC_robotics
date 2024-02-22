package org.firstinspires.ftc.teamcode.autos;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "UNHINGED_RED_AUTO_CYCLE", group = "Autonomous")
public class RedSideAutoUnhinged extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    // Claw Declarations
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.67);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenPickupClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.52);
                return false;
            }
        }
        public Action openPickupClaw() {
            return new OpenPickupClaw();
        }
        public class OpenDropOffClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.58);
                return false;
            }
        }
        public Action openDropOfflaw() {
            return new OpenDropOffClaw();
        }
    }

    // hopper declarations
    public class Hopper {
        private Servo hopper;

        public Hopper(HardwareMap hardwareMap) {
            hopper = hardwareMap.get(Servo.class, "hopper");
        }

        public class CloseHopper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hopper.setPosition(0.38);
                return false;
            }
        }
        public Action closeHopper() {
            return new CloseHopper();
        }

        public class OpenHopper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hopper.setPosition(0.05);
                return false;
            }
        }
        public Action openHopper() {
            return new OpenHopper();
        }
    }
    // wrist declarations
    public class Wrist {
        private Servo leftwrist;
        private Servo rightwrist;

        public Wrist(HardwareMap hardwareMap) {
            leftwrist = hardwareMap.get(Servo.class, "leftwrist");
            rightwrist = hardwareMap.get(Servo.class, "rightwrist");
        }

        public class LiftWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.13);
                rightwrist.setPosition(1 - 0.13);
                return false;
            }
        }
        public Action liftWrist() {
            return new LiftWrist();
        }

        public class LowerWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.825);
                rightwrist.setPosition(1 - 0.825);
                return false;
            }
        }
        public Action lowerWrist() {
            return new LowerWrist();
        }
        public class SafeWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.6);
                rightwrist.setPosition(1 - 0.6);
                return false;
            }
        }
        public Action safeWrist() {
            return new SafeWrist();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .build();
        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction2;
        } else {
            trajectoryActionChosen = trajectoryAction3;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}