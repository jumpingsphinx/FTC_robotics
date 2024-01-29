package org.firstinspires.ftc.teamcode.utils;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Airplane Position Finder", group = "TeleOp")
public class AirplanePositionFinder extends LinearOpMode {
    private Servo launcher;

    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(Servo.class, "launcher");
        double position = 0.75;
        double increment = -0.03;
        launcher.setPosition(0.85);

        waitForStart();

        while (opModeIsActive()) {
            // sweep
            launcher.setPosition(position);

            telemetry.addData("Position", position);
            telemetry.update();
            sleep(3000);

            position += increment;

            if (position > 1.0) {
                break;
            }
        }
        launcher.setPosition(0.5);

        telemetry.addData("Status", "Calibration Complete");
        telemetry.update();
        sleep(2000);
    }
}
