//package org.firstinspires.ftc.teamcode.autos;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.Pose2d;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.acmerobotics.roadrunner.Action;
//
////public class Drive {
////    public Action followTrajectory(Trajectory t) {
////        return new TodoAction();
////    }
////
////    public Action turn(double angle) {
////        return new TodoAction();
////    }
////
////    public Action moveToPoint(double x, double y) {
////        return new TodoAction();
////    }
////}
//
//@Autonomous(name="Red Vision Auto", group="Linear Opmode")
//public class RedVisionAuto extends LinearOpMode {
//
//    private MecanumDrive drive;
//
//    @Override
//    public void runOpMode() {
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        // CAMERA LOGIC HERE
//        int element = 1; // or 2, or 3
//
//        // Define the coordinates for (x1, y1) and (x2, y2) based on the value
//        Vector2d firstPosition = getFirstPosition(element);
//        Vector2d secondPosition = getSecondPosition(element);
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Build and follow the trajectory to the first position
//
//            Trajectory trajectoryToFirst = drive.trajectoryBuilder(new Pose2d())
//                    .splineTo(firstPosition, 0)
//                    .build();
//            drive.followTrajectory(trajectoryToFirst);
//
//            // Pause for 3 seconds
//            sleep(3000);
//
//            // Move backward by 3 cm
//            drive.followTrajectory(
//                    drive.trajectoryBuilder(trajectoryToFirst.end())
//                            .back(3)
//           // Move to (x2, y2)
//            drive.followTrajectory(
//                    drive.trajectoryBuilder(new Pose2d(firstPosition.x, firstPosition.y - 3, 0))
//                            .splineTo(secondPosition, Math.toRadians(90)) // Rotating 90 degrees
//                            .build()
//            );
//
//            // Wait for 5 seconds
//            sleep(5000);
//        }
//    }
//
//    private Vector2d                     .build()
//            );
//
//    getFirstPosition(int value) {
//        // Return the first position based on the value
//        // Example: return new Vector2d(x1, y1);
//    }
//
//    private Vector2d getSecondPosition(int value) {
//        // Return the second position based on the value
//        // Example: return new Vector2d(x2, y2);
//    }
//}
