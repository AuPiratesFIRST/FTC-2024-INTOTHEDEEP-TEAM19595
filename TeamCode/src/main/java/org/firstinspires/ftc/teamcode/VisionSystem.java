package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.vision.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.vision.VisionPortal;

public class VisionSystem {

    // Vision processing objects
    private VisionPortal myVisionPortal;
    private AprilTagProcessor myAprilTagProcessor;
    private TfodProcessor myTfodProcessor;

    // Constructor to initialize vision system
    public VisionSystem(WebcamName webcam) {
        // Initialize the AprilTag processor
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Initialize the TFOD processor
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();

        // Create and initialize the vision portal
        myVisionPortal = new VisionPortal.Builder()
                .addProcessor(myAprilTagProcessor)
                .addProcessor(myTfodProcessor)
                .build();

        // Enable the vision portal
        myVisionPortal.enable();
    }

    // Start vision processing
    public void startVision() {
        myVisionPortal.start();
    }

    // Stop vision processing
    public void stopVision() {
        myVisionPortal.stop();
    }

    // Get last detected AprilTag data
    public AprilTagProcessor.DetectionData getAprilTagData() {
        return myAprilTagProcessor.getLastDetectedTags();
    }

    // Get TFOD recognition data
    public TfodProcessor.Recognition getTfodData() {
        return myTfodProcessor.getRecognitions().isEmpty() ? null : myTfodProcessor.getRecognitions().get(0);
    }
}
