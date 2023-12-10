package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.osgi.OpenCVInterface;

public class Tester {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args) {
        VideoCapture camera = new VideoCapture(0); // 0 is the id of the default camera

        if (!camera.isOpened()) {
            System.out.println("Error: Could not open the camera.");
            return;
        }

        Mat frame = new Mat();
        BlueVisionPipeline pipeline = new BlueVisionPipeline();

        while (true) {
            if (camera.read(frame)) {
                pipeline.processFrame(frame);
                int position = pipeline.outputPosition();
                System.out.println("Position: " + position);

                // Display the frame if you want (using a HighGui window, for example)
                // HighGui.imshow("Webcam", frame);
                // if (HighGui.waitKey(30) >= 0) break;
            } else {
                System.out.println("Error: Could not read frame from camera.");
                break;
            }
        }

        camera.release();
        // HighGui.destroyAllWindows();
    }
}
