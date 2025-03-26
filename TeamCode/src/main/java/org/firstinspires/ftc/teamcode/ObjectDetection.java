package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;

public class ObjectDetection {

    // Camera dimensions
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    // Real-world dimensions of the object
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.5; // Example width in inches

    private OpenCvCamera camera;
    public ObjectDetectionPipeline objectDetectionPipeline;

    // Constructor
    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        objectDetectionPipeline = new ObjectDetectionPipeline();
        startObjectDetectionPipeline(hardwareMap); // Start the camera pipeline
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    // Initialize and start the camera
    private void startObjectDetectionPipeline(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(objectDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Camera opened successfully, now start streaming
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera error if needed
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }


    // Cleanup method to stop the camera and release resources
    public void stopObjectDetection() {
        if (camera != null) {
            camera.closeCameraDevice();
        }
    }

    // Optional: Telemetry updates for debugging and visualization
    public void updateTelemetry() {
        telemetry.addData("Centroid X", objectDetectionPipeline.getCentroidX());
        telemetry.addData("Centroid Y", objectDetectionPipeline.getCentroidY());
        telemetry.addData("Object Width", objectDetectionPipeline.getObjectWidth());
        telemetry.addData("Distance", objectDetectionPipeline.getDistance());
        telemetry.update();
    }
}
