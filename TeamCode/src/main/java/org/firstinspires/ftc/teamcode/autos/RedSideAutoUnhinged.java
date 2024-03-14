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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "RED_NEAR_FULL_RR", group = "Autonomous")
public class RedSideAutoUnhinged extends LinearOpMode {
    private static ElapsedTime myStopwatch = new ElapsedTime();
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
                    lift.setPower(0.6);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 900.0) {
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
        public class LiftUpAgain implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.6);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1700.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUpAgain() {
            return new LiftUpAgain();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    sleep(200);
                    lift.setPower(-0.4);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 10.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    sleep(400);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    //pullup declarations
    public class PullUp {
        private DcMotorEx pullupright;
        private DcMotorEx pullupleft;

        public PullUp(HardwareMap hardwareMap) {
            pullupleft = hardwareMap.get(DcMotorEx.class, "pullupleft");
            pullupleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pullupleft.setDirection(DcMotor.Direction.FORWARD);
            pullupright = hardwareMap.get(DcMotorEx.class, "pullupright");
            pullupright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pullupright.setDirection(DcMotor.Direction.REVERSE);
        }

        public class PullUpDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pullupleft.setPower(-0.6);
                    pullupright.setPower(-0.6);
                    myStopwatch.reset();
                    initialized = true;
                }

                packet.put("pullup Status: ", "inited");
                if (myStopwatch.time(TimeUnit.MILLISECONDS) < 300.0) {
                    return true;
                } else {
                    pullupright.setPower(0);
                    pullupleft.setPower(0);
                    return false;
                }
            }
        }
        public Action pullUpDown() {
            return new PullUpDown();
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
                claw.setPosition(0.75);
                sleep(350);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenPickupClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.56);
                sleep(350);
                return false;
            }
        }
        public Action openPickupClaw() {
            return new OpenPickupClaw();
        }
        public class OpenDropOffClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.60);
                sleep(450);
                return false;
            }
        }
        public Action openDropOffClaw() {
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
                sleep(350);
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
                sleep(450);
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
            leftwrist = hardwareMap.get(Servo.class, "wristleft");
            rightwrist = hardwareMap.get(Servo.class, "wristright");
        }

        public class LiftWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.15);
                rightwrist.setPosition(1 - 0.15);
                sleep(350);
                return false;
            }
        }
        public Action liftWrist() {
            return new LiftWrist();
        }

        public class LowerWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.85);
                rightwrist.setPosition(1 - 0.85);
                sleep(1200);
                return false;
            }
        }
        public Action lowerWrist() {
            return new LowerWrist();
        }
        public class SafeWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftwrist.setPosition(0.63);
                rightwrist.setPosition(1 - 0.63);
                sleep(350);
                return false;
            }
        }
        public Action safeWrist() {
            return new SafeWrist();
        }
    }

    // begin camera stuff
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240119_145409.tflite";

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    private static final String[] LABELS = {
            "cone"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, -61.7, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Hopper hopper = new Hopper(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        PullUp pullUp = new PullUp(hardwareMap);

        Action trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(-37)
                .turnTo(Math.toRadians(183))
                .setTangent(Math.toRadians(90))
                .lineToY(-34.5)
                .build();
        Action trajectoryAction1Pt2 = drive.actionBuilder(new Pose2d(14,-33,Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToX(44.5)
                .setTangent(Math.toRadians(90))
                .lineToY(-26)
                .setTangent(0)
                .lineToX(50)
                .build();
        Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(-39.5)
                .setTangent(Math.toRadians(0))
                .lineToX(16)
                .build();
        Action trajectoryAction2Pt2 = drive.actionBuilder(new Pose2d(18, -37, Math.toRadians(90)))
                .lineToY(-40)
                .turnTo(Math.toRadians(183))
                .setTangent(Math.toRadians(0))
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-31.5)
                .setTangent(Math.toRadians(0))
                .lineToX(49.5)
                .build();
        Action trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToY(-51.5)
                .setTangent(0)
                .lineToX(23)
                .build();
        Action trajectoryAction3Pt2 = drive.actionBuilder(new Pose2d(23, -50.5, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-56.5)
                .turnTo(Math.toRadians(177))
                .setTangent(Math.toRadians(0))
                .lineToX(40)
                .setTangent(Math.toRadians(90))
                .lineToY(-36)
                .setTangent(Math.toRadians(0))
                .lineToX(51)
                .build();
        Action trajectoryActionCloseOut3 = drive.actionBuilder(new Pose2d(51, -44, Math.toRadians(180)))
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                .setTangent(0)
                .lineToX(54)
                .build();
        Action trajectoryActionCloseOut1 = drive.actionBuilder(new Pose2d(50, -27, Math.toRadians(180)))
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                .setTangent(0)
                .lineToX(54)
                .build();
        Action trajectoryActionCloseOut2 = drive.actionBuilder(new Pose2d(49.5, -31.5, Math.toRadians(180)))
                .lineToX(45)
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                .setTangent(0)
                .lineToX(54)
                .build();
        Action trajectoryActionCyclePartOne = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-53.9, 11.5))
                .build();
        //PICKUP
        Action trajectoryActionAfterFirstPickup = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 11))
                .strafeToConstantHeading(new Vector2d(49, 28.9))
                .build();
        //PLACE
        Action trajectoryActionAfterFirstPlace = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 11))
                .strafeTo(new Vector2d(-53.9, 11.5))
                .build();
        //PICKUP
        Action trajectoryActionAfterSecondPickup = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 11))
                .strafeToConstantHeading(new Vector2d(49, 28.9))
                .build();
        //PLACE

        Action purplePixelPlace = new SequentialAction(
                wrist.lowerWrist(),
                claw.openPickupClaw(),
                wrist.safeWrist(),
                claw.closeClaw(),
                wrist.liftWrist()
        );

        Action pixelPlace = new SequentialAction(
                wrist.lowerWrist(),
                lift.liftUp(),
                hopper.openHopper(),
                lift.liftUpAgain(),
                lift.liftDown(),
                hopper.closeHopper(),
                wrist.liftWrist()
        );
        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());

        int pos = 3;
        while (!opModeIsActive()) {
            pos = telemetryTfod();
            telemetry.addData("position: ", pos);
            telemetry.update();
            sleep(20);
        }
        int startPosition = pos;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        Action trajectoryActionChosenPt2;
        Action trajectoryActionCloseOut;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
            trajectoryActionChosenPt2 = trajectoryAction1Pt2;
            trajectoryActionCloseOut = trajectoryActionCloseOut1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction2;
            trajectoryActionChosenPt2 = trajectoryAction2Pt2;
            trajectoryActionCloseOut = trajectoryActionCloseOut2;
        } else {
            trajectoryActionChosen = trajectoryAction3;
            trajectoryActionChosenPt2 = trajectoryAction3Pt2;
            trajectoryActionCloseOut = trajectoryActionCloseOut3;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        purplePixelPlace,
                        trajectoryActionChosenPt2,
                        pixelPlace,
                        trajectoryActionCloseOut,
                        pullUp.pullUpDown()
                )
        );
        visionPortal.close();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public int telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        int position = 3;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x <= 200) {
                position = 1;
            } else if (x < 460 && x > 200) {
                position = 2;
            }
            telemetry.addData("position", position);
        }   // end for() loop

        return position;
    }

}