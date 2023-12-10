package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueVisionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Denoise the image
        Imgproc.GaussianBlur(input, mat, new Size(5, 5), 0);

        // Convert to HSV
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // Define blue color range and create a mask
        Scalar lowerBlue = new Scalar(100, 150, 0); // Adjust these values
        Scalar upperBlue = new Scalar(140, 255, 255); // Adjust these values
        Core.inRange(mat, lowerBlue, upperBlue, mat);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour and its position (left, center, right)
        double maxArea = 0;
        Rect largestRect = new Rect();
        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea > maxArea) {
                maxArea = contourArea;
                largestRect = Imgproc.boundingRect(contour);
            }
        }

        if (maxArea > 0) {
            // Draw the bounding box of the largest contour
            Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), new Scalar(0, 255, 0), 2);

            int widthThird = input.width() / 3;
            int centerX = largestRect.x + largestRect.width / 2;

            String position;
            if (centerX < widthThird) {
                position = "Left";
            } else if (centerX < 2 * widthThird) {
                position = "Center";
            } else {
                position = "Right";
            }

            // Optionally, display the position on the screen
            Imgproc.putText(input, position, new Point(10, input.height() - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 2);
        }

        return input;
    }
}
