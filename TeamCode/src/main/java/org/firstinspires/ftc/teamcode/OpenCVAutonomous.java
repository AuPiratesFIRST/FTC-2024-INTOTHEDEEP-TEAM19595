package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV Autonomous")
public class OpenCVAutonomous extends LinearOpMode {
    private ObjectDetection objectDetection;
    private Tracking tracking;  // Renamed from RobotMovement to Tracking

    @Override
    public void runOpMode() {
        // Initialize object detection system and tracking system
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        tracking = new Tracking(hardwareMap, telemetry);  // Instantiate Tracking class

        // Set up telemetry to display data to both the Driver Station and the FtcDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start streaming camera feed to dashboard
        dashboard.startCameraStream(objectDetection.getCamera(), 30); // Stream the camera from ObjectDetection
        // Wait for the start of the autonomous period
        waitForStart();

        // Autonomous loop
        while (opModeIsActive()) {
            double detectedWidth = objectDetection.objectDetectionPipeline.getObjectWidth();
            double distance = objectDetection.objectDetectionPipeline.getDistance();

            // Use object detection to control the robot
            if (detectedWidth > 0) {
                telemetry.addData("Distance to object (inches)", distance);
                telemetry.update();

                // Move towards the object using Tracking
                tracking.moveTowardsObject(distance);
            } else {
                telemetry.addData("Status", "No object detected");
                telemetry.update();

                // If no object is detected, search for it
                tracking.searchForObject();
            }
        }

        // Stop robot movement after autonomous period ends
        tracking.stopMovement();
    }
}
