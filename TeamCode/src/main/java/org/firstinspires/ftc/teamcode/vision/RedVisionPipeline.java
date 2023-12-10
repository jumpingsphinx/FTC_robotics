package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedVisionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    private int position = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Denoise the image
        Imgproc.GaussianBlur(input, mat, new Size(5, 5), 0);

        // Convert to HSV
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // Define red color range and create a mask
        Scalar lowerBlue = new Scalar(0, 75, 50);
        Scalar upperBlue = new Scalar(9, 90, 70);
        Mat mask1 = new Mat();
        Core.inRange(mat, lowerBlue, upperBlue, mask1);

        // Define second red color range and create a mask
        Scalar lowerRed2 = new Scalar(160, 75, 50); // Adjust these values
        Scalar upperRed2 = new Scalar(180, 90, 85); // Adjust these values
        Mat mask2 = new Mat();
        Core.inRange(mat, lowerRed2, upperRed2, mask2);

        // Combine both masks
        Core.add(mask1, mask2, mat);
        mask1.release();
        mask2.release();

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
