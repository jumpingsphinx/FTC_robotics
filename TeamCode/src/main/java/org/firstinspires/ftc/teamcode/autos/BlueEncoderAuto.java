package org.firstinspires.ftc.teamcode.autos;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.ConceptTensorFlowObjectDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.List;


@Config
@Autonomous(name = "BLUE_PURPLE_AUTO", group = "Autonomous")
@Disabled
public class BlueEncoderAuto extends LinearOpMode {

    public static final double DRIVER_SPEED_SCALAR = 0.85;
    public static final double DRIVER_SPRINT_MODE_SCALAR = 0.95;
    public static final double DRIVER_ROTATION_SCALAR = 0.7;
    public static final double DRIVER_SLOW_MODE_SCALAR = 0.50;
    public static final double SENSITIVITY_THRESHOLD = 0.20;
    public static final double LIFT_SCALAR = 0.85;
    public static final double PULL_SCALAR = 0.95;
    public static final double COURSE_CORRECT = 1.04;

    //GUNNER CONSTANTS
    public static final double HOPPER_OPEN = 0.05;
    public static final double HOPPER_CLOSED = 0.38;
    public static final double CLAW_CLOSED = 0.66;
    public static final double CLAW_OPEN_PICKUP = 0.52;
    public static final double CLAW_OPEN_DROPOFF = 0.60;
    public static final double WRIST_UP = 0.13;
    public static final double WRIST_DOWN = 0.825;
    public static final double LAUNCHER_HOLD = 0.85;
    public static final double LAUNCHER_RELEASE = 0.68;

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

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20240119_145409.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "cone"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Save more CPU resources when camera is no longer needed.
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        lift = hardwareMap.get(DcMotor.class, "lift");
        pullupleft = hardwareMap.get(DcMotor.class, "pullupleft");
        pullupright = hardwareMap.get(DcMotor.class, "pullupright");

        wristleft = hardwareMap.get(Servo.class, "wristleft");
        wristright = hardwareMap.get(Servo.class, "wristright");
        claw = hardwareMap.get(Servo.class, "claw");
        hopper = hardwareMap.get(Servo.class, "hopper");
        launcher = hardwareMap.get(Servo.class, "launcher");
        claw.setPosition(CLAW_CLOSED);
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

        pullupleft.setDirection(DcMotor.Direction.FORWARD);
        pullupright.setDirection(DcMotorSimple.Direction.REVERSE);
        pullupleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullupright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Wrapper wrapMotors = new Wrapper (fl,fr,bl,br);

        int pos = 3;
        while (!opModeIsActive()) {
            pos = telemetryTfod();
            telemetry.addData("position: ", pos);
            telemetry.update();
            sleep(20);
        }
        waitForStart();

        if (opModeIsActive()){
        if (pos == 3){
            wrappypoo(-175, 182, 160, -131);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(-175,182,160,-131));
            sleep(1000);
            wrappypoo(711,1037,1064,754);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(711,1037,1064,754));
            sleep(1000);
            wrappypoo(1485,1789,227,-122);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(1485,1789,227,-122));
            sleep(1000);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(1543,1837,268,-90));
            wrappypoo(1543,1837,268,-90);
            sleep(1000);
//            telemetry.addLine(purpleDropSequence());
            purpleDropSequence();
        }
        else if (pos == 2){
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(-203,221,207,-201));
            wrappypoo(-203,221,207,-201);
            sleep(1000);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(549,967,1019,589));
            wrappypoo(525,925,975,550);
            sleep(1000);
//            telemetry.addLine(purpleDropSequence());
            purpleDropSequence();
        }
        // handle pos = 1
        else {
            wrappypoo(-445, 405, 444, -402);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(-445, 405, 444, -402));
            sleep(1000);
            wrappypoo(3, 827, 875, 42);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(3, 827, 875, 42));
            sleep(1000);
//            telemetry.addLine(purpleDropSequence());
            purpleDropSequence();
        }
        }
        visionPortal.close();
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private String purpleDropSequence() {
        wristleft.setPosition(WRIST_DOWN);
        wristright.setPosition(1 - WRIST_DOWN);
        sleep(1000);
        claw.setPosition(CLAW_OPEN_PICKUP);
        sleep(1000);
        wrappypoo(fl.getCurrentPosition() - 50, bl.getCurrentPosition() - 50, fr.getCurrentPosition() - 50, br.getCurrentPosition() - 50);
        sleep(500);
        wristleft.setPosition(WRIST_UP);
        wristright.setPosition(1 - WRIST_UP);
        sleep(200);
        claw.setPosition(CLAW_CLOSED);
        sleep(1000);
        return "Placed Purple!";
    }
    private String yellowDropSequence() {
        hopper.setPosition(HOPPER_CLOSED);
        wristleft.setPosition(WRIST_DOWN);
        wristright.setPosition(1 - WRIST_DOWN);
        lift.setPower(0.6);
        sleep(1000);
        lift.setPower(0);
        hopper.setPosition(HOPPER_OPEN);
        sleep(500);
        hopper.setPosition(HOPPER_CLOSED);
        lift.setPower(-0.6);
        sleep(1000);
        lift.setPower(0);
        return "Placed Yellow!";
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
    public void wrappypoo(int flticks, int blticks, int frticks,  int brticks){
        fl.setTargetPosition(flticks);
        bl.setTargetPosition(blticks);
        fr.setTargetPosition(frticks);
        br.setTargetPosition(brticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double max = Math.max(Math.abs(flticks), Math.max(Math.abs(frticks), Math.max(Math.abs(blticks), Math.abs(brticks))));
        double flticksscaled = flticks / max;
        double frticksscaled = frticks / max;
        double blticksscaled = blticks / max;
        double brticksscaled = brticks / max;


        fl.setPower(0.4);
        bl.setPower(0.4);
        fr.setPower(0.4);
        br.setPower(0.4);

        while (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()){
            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.addData("fltarget", fl.getTargetPosition());
            telemetry.addData("bltarget", bl.getTargetPosition());
            telemetry.addData("frtarget", fr.getTargetPosition());
            telemetry.addData("brtarget", br.getTargetPosition());
            telemetry.update();
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

    }

