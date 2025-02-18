package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.List;
import java.util.ArrayList;

public class ObjectDetectionPipeline extends OpenCvPipeline {

    private Mat hsvImage = new Mat();
    private Mat blurredImage = new Mat();
    private Mat thresholdRed = new Mat();
    private Mat thresholdBlue = new Mat();
    private Mat thresholdYellow = new Mat();
    private Mat combinedThreshold = new Mat();

    private Scalar lowerRed = RedColorDetection.lowerRed;
    private Scalar upperRed = RedColorDetection.upperRed;
    private Scalar lowerBlue = BlueColorDetection.lowerBlue;
    private Scalar upperBlue = BlueColorDetection.upperBlue;
    private Scalar lowerYellow = YellowColorDetection.lowerYellow;
    private Scalar upperYellow = YellowColorDetection.upperYellow;

    public double cX = 0;
    public double cY = 0;
    public double width = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvImage, lowerRed, upperRed, thresholdRed);
        Core.inRange(hsvImage, lowerBlue, upperBlue, thresholdBlue);
        Core.inRange(hsvImage, lowerYellow, upperYellow, thresholdYellow);

        Core.bitwise_or(thresholdRed, thresholdBlue, combinedThreshold);
        Core.bitwise_or(combinedThreshold, thresholdYellow, combinedThreshold);

        Imgproc.GaussianBlur(combinedThreshold, blurredImage, new Size(5, 5), 0);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blurredImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            double aspectRatio = (double) boundingRect.width / boundingRect.height;

            if (boundingRect.width > 50 && boundingRect.height > 50 && aspectRatio > 0.5 && aspectRatio < 2.0) {
                Moments moments = Imgproc.moments(contour);
                if (moments.get_m00() != 0) {
                    cX = moments.get_m10() / moments.get_m00();
                    cY = moments.get_m01() / moments.get_m00();
                }
                width = boundingRect.width;

                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(input, "X: " + (int) cX + " Y: " + (int) cY, new Point(cX, cY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);
            }
        }

        return input;
    }

    public double getDistance() {
        if (width == 0) return Double.MAX_VALUE;
        return (ObjectDetection.OBJECT_WIDTH_IN_REAL_WORLD_UNITS * ObjectDetection.CAMERA_WIDTH) / width;
    }

    public double getCentroidX() {
        return cX;
    }

    public double getCentroidY() {
        return cY;
    }

    public double getWidth() {
        return width;
    }
}
