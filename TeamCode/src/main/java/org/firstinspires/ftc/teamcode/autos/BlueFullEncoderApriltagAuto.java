package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.ArrayList;


@Config
@Autonomous(name = "BLUE_FULL_APRILTAG_AUTO", group = "Autonomous")
@Disabled
public class BlueFullEncoderApriltagAuto extends LinearOpMode {

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
    public static final double LAUNCHER_HOLD = 0.83;
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
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;


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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);


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
                sleep(200);
                wrappypoo(711,1037,1064,754);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(711,1037,1064,754));
                sleep(200);
                wrappypoo(1485,1789,227,-122);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(1485,1789,227,-122));
                sleep(200);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(1543,1837,268,-90));
                wrappypoo(1543,1837,268,-90);
                sleep(200);
//            telemetry.addLine(purpleDropSequence());
                purpleDropSequence();
                wrappypoo(338,631,-923,-1292);
                sleep(200);
                wrappypoo(132,806,-763,-1491);
                sleep(200);
                apriltagthingy(3);

            }
            else if (pos == 2){
                int ID_TAG_OF_INTEREST = 2;
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(-203,221,207,-201));
                wrappypoo(-203,221,207,-201);
                sleep(200);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(549,967,1019,589));
                wrappypoo(525,925,975,550);
                sleep(200);
//            telemetry.addLine(purpleDropSequence());
                purpleDropSequence();
                wrappypoo(1300,1730,202,-291);
                sleep(200);
                wrappypoo(140,580,-943,-1430);
                sleep(200);
                apriltagthingy(3);
            }
            // handle pos = 1
            else {
                int ID_TAG_OF_INTEREST = 1;
                wrappypoo(-445, 405, 444, -402);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(-445, 405, 444, -402));
                sleep(200);
                wrappypoo(3, 827, 875, 42);
//            telemetry.addData("fl, fr, bl, br", wrapMotors.driveToEncoderPosition(3, 827, 875, 42));
                sleep(200);
//            telemetry.addLine(purpleDropSequence());
                purpleDropSequence();
                wrappypoo(698,1492,7,-842);
                sleep(200);
                wrappypoo(42,825,-688,-1530);
                sleep(200);
                wrappypoo(-221,1141,-438,-1829);
                sleep(200);
                wrappypoo(-435,887,-664,-2049);
                sleep(200);
                apriltagthingy(3);
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
        sleep(500);
        wrappypoo(fl.getCurrentPosition() - 50, bl.getCurrentPosition() - 50, fr.getCurrentPosition() - 50, br.getCurrentPosition() - 50);
        sleep(500);
        wristleft.setPosition(WRIST_UP);
        wristright.setPosition(1 - WRIST_UP);
        sleep(200);
        claw.setPosition(CLAW_CLOSED);
        sleep(200);
        return "Placed Purple!";
    }
    private String yellowDropSequence() {
        hopper.setPosition(HOPPER_CLOSED);
        wristleft.setPosition(WRIST_DOWN);
        wristright.setPosition(1 - WRIST_DOWN);
        sleep(500);
        lift.setPower(0.6);
        sleep(2000);
        lift.setPower(0);
        sleep(1000);
        hopper.setPosition(HOPPER_OPEN);
        sleep(1000);
        hopper.setPosition(HOPPER_CLOSED);
        lift.setPower(-0.6);
        sleep(2000);
        lift.setPower(0);
        return "Placed Yellow!";
    }
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();

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
    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
    public void apriltagthingy(int ID_TAG_OF_INTEREST){
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == ID_TAG_OF_INTEREST)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

            if(tagOfInterest.pose.x < 0.1)
            {
                // move one way
            }
            else if(tagOfInterest.pose.x > 0.1)
            {
                // move the other way
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                yellowDropSequence();
            }
        }
    }

}

