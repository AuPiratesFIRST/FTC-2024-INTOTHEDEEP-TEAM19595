//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.robotcore.external.tfod.TfodProcessor;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.ArrayList;
//
//public class VisionSystem {
//
//    // Vision processing objects
//    private VisionPortal myVisionPortal;
//    private AprilTagProcessor myAprilTagProcessor;
//   // private TfodProcessor myTfodProcessor;
//
//    // Constructor to initialize vision system
//    public VisionSystem(WebcamName cam) {
//        // Initialize the AprilTag processor
//        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//
//        // Initialize the TFOD processor
//      //  myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
//
//        // Create and initialize the vision portal
//        myVisionPortal = new VisionPortal.Builder()
//                .addProcessor(myAprilTagProcessor)
//             //   .addProcessor(myTfodProcessor)
//                .build();
//
//        // Enable the vision portal
//      //  myVisionPortal.setProcessorEnabled(ture);
//    }
//
//    // Start vision processing
//    public void startVision() {
////        myVisionPortal.setActiveCamera(WebcamName myAprilTagProcessor );
////    }
////
//     Stop vision processing
////    public void stopVision() {
////        myVisionPortal.stopStreaming();
////    }
////
//     Get last detected AprilTag data
////    public ArrayList<AprilTagDetection> getAprilTagData() {
////        return myAprilTagProcessor.getDetections();
////    }
////
//     Get TFOD recognition data
//    public TfodProcessor.Recognition getTfodData() {
//        return myTfodProcessor.getRecognitions().isEmpty() ? null : myTfodProcessor.getRecognitions().get(0);
//    }
////}
