package org.firstinspires.ftc.teamcode; // Adjust package name as needed

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Import Telemetry

import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionPipeline extends OpenCvPipeline {

    // Constants for filtering contours
    private static final int MIN_CONTOUR_WIDTH = 50;
    private static final int MIN_CONTOUR_HEIGHT = 50;
    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;

    // Region of Interest (ROI) - Define your box here
    private static final Rect ROI = new Rect(100, 100, 200, 200); // Example: x=100, y=100, width=200, height=200

    // Color ranges (using static inner classes for better organization)
    public static class ColorRange {
        public final Scalar lower;
        public final Scalar upper;

        public ColorRange(Scalar lower, Scalar upper) {
            this.lower = lower;
            this.upper = upper;
        }
    }

    public static class RedColorRange {
        public static final ColorRange RANGE = new ColorRange(new Scalar(0, 100, 100), new Scalar(10, 255, 255)); // Example range
    }

    public static class BlueColorRange {
        public static final ColorRange RANGE = new ColorRange(new Scalar(100, 100, 100), new Scalar(140, 255, 255)); // Example range
    }

    public static class YellowColorRange {
        public static final ColorRange RANGE = new ColorRange(new Scalar(20, 100, 100), new Scalar(40, 255, 255)); // Example range
    }

    // Reusable Mat objects
    private final Mat hsvImage = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat thresholdedImage = new Mat();
    private Mat roiMat = new Mat();

    // Color ranges
    private final ColorRange redRange = RedColorRange.RANGE;
    private final ColorRange blueRange = BlueColorRange.RANGE;
    private final ColorRange yellowRange = YellowColorRange.RANGE;

    // Object properties
    private double centroidX = 0;
    private double centroidY = 0;
    private double objectWidth = 0;

    // Telemetry for debugging
    private Telemetry telemetry;

    // Constructor to pass Telemetry
    public ObjectDetectionPipeline() {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Define the ROI
        roiMat.release();
        roiMat = new Mat(input, ROI);

        // Convert to HSV color space (only for the ROI)
        Imgproc.cvtColor(roiMat, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Combine color thresholds
        Mat redThreshold = new Mat();
        Mat blueThreshold = new Mat();
        Mat yellowThreshold = new Mat();

        Core.inRange(hsvImage, redRange.lower, redRange.upper, redThreshold);
        Core.inRange(hsvImage, blueRange.lower, blueRange.upper, blueThreshold);
        Core.inRange(hsvImage, yellowRange.lower, yellowRange.upper, yellowThreshold);

        Core.bitwise_or(redThreshold, blueThreshold, thresholdedImage);
        Core.bitwise_or(thresholdedImage, yellowThreshold, thresholdedImage);

        // Release temporary Mats
        redThreshold.release();
        blueThreshold.release();
        yellowThreshold.release();

        // Apply Gaussian blur
        Imgproc.GaussianBlur(thresholdedImage, blurredImage, new Size(5, 5), 0);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blurredImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process contours
        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            double aspectRatio = (double) boundingRect.width / boundingRect.height;

            // Filter contours based on size and aspect ratio
            if (boundingRect.width > MIN_CONTOUR_WIDTH && boundingRect.height > MIN_CONTOUR_HEIGHT &&
                    aspectRatio > MIN_ASPECT_RATIO && aspectRatio < MAX_ASPECT_RATIO) {
                Moments moments = Imgproc.moments(contour);
                if (moments.get_m00() != 0) {
                    centroidX = moments.get_m10() / moments.get_m00();
                    centroidY = moments.get_m01() / moments.get_m00();
                }
                objectWidth = boundingRect.width;

                // Adjust centroid coordinates to be relative to the full image
                centroidX += ROI.x;
                centroidY += ROI.y;

                // Draw on the input image
                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                Imgproc.rectangle(input, new Point(ROI.x, ROI.y), new Point(ROI.x + ROI.width, ROI.y + ROI.height), new Scalar(0, 255, 255), 2);
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(input, "X: " + (int) centroidX + " Y: " + (int) centroidY, new Point(centroidX, centroidY),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);
                // We found the object, so we can stop looking for other contours
                break;
            }
        }
        // Add telemetry data
        telemetry.addData("Object Width", objectWidth);
        telemetry.addData("Centroid X", centroidX);
        telemetry.addData("Centroid Y", centroidY);
        telemetry.update();

        return input;
    }

    public double getDistance() {
        if (objectWidth == 0) return Double.MAX_VALUE;
        return (ObjectDetection.OBJECT_WIDTH_IN_REAL_WORLD_UNITS * ObjectDetection.CAMERA_WIDTH) / objectWidth;
    }

    public double getCentroidX() {
        return centroidX;
    }

    public double getCentroidY() {
        return centroidY;
    }

    public double getObjectWidth() {
        return objectWidth;
    }
}