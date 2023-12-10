package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueVisionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    private int position = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Denoise the image
        Imgproc.GaussianBlur(input, mat, new Size(5, 5), 0);

        // Convert to HSV
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // Define blue color range and create a mask
        Scalar lowerBlue = new Scalar(10, 14, 33);
        Scalar upperBlue = new Scalar(45, 70, 200);
        Core.inRange(mat, lowerBlue, upperBlue, mat);

        int widthThird = input.width() / 3;
        int[] blueCounts = new int[3];

        for (int i = 0; i < 3; i++) {
            // Define the region of interest for each third
            Mat region = mat.submat(0, mat.rows(), i * widthThird, (i + 1) * widthThird);
            blueCounts[i] = Core.countNonZero(region);
            region.release();
        }

        // Determine the position with the maximum blue pixel count
        if (blueCounts[0] > blueCounts[1] && blueCounts[0] > blueCounts[2]) {
            position = 1; // Left third has the most blue
        } else if (blueCounts[1] > blueCounts[2]) {
            position = 2; // Center third has the most blue
        } else {
            position = 3; // Right third has the most blue
        }

        return input;
    }

    public int outputPosition() {
        return position;
    }
}
