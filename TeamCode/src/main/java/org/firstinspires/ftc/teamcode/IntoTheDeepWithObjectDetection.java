package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.*;
import java.util.List;

@Autonomous(name="IntoTheDeepWithObjectDetection")
public class IntoTheDeepWithObjectDetection extends LinearOpMode {

    private RobotHardware robot;
    private ObjectDetection objectDetection;
    private AprilTagDetection aprilTagDetection;

    @Override
    public void runOpMode() {
        // Initialize hardware and vision
        robot = new RobotHardware(hardwareMap, telemetry);
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        aprilTagDetection = new AprilTagDetection(); // Initialize the AprilTagDetection class

        // Initialize FtcDashboard for real-time monitoring
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start streaming camera feed to dashboard
        dashboard.startCameraStream(objectDetection.getCamera(), 30); // Stream the camera from ObjectDetection

        // Initialize AprilTag detection
        aprilTagDetection.init(hardwareMap); // Initialize the AprilTagDetection instance

        // Wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            // Perform object detection
            while (opModeIsActive()) {
                // Object detection pipeline is running automatically; get the data directly
                double objectCentroidX = objectDetection.objectDetectionPipeline.getCentroidX();
                double objectCentroidY = objectDetection.objectDetectionPipeline.getCentroidY();
                double objectDistance = objectDetection.objectDetectionPipeline.getDistance();
                double objectWidth = objectDetection.objectDetectionPipeline.getObjectWidth();

                // Display object detection info in telemetry
                telemetry.addData("Object Centroid X", objectCentroidX);
                telemetry.addData("Object Centroid Y", objectCentroidY);
                telemetry.addData("Object Distance (inches)", objectDistance);
                telemetry.addData("Object Width (pixels)", objectWidth);
                telemetry.update();

                // Behavior based on object distance
                if (objectDistance < 24) {
                    telemetry.addData("Status", "Object too close! Stopping.");
                    telemetry.update();
                    robot.stopMotors(); // Stop moving if too close
                    sleep(1000); // Wait for a second for the servo to open (if needed)
                    break; // End loop if object is too close
                } else {
                    // Otherwise, move forward towards the object
                    telemetry.addData("Status", "Moving forward.");
                    telemetry.update();
                    robot.setMotorPower(0.5, 0.5); // Move forward at 50% power
                }

                // If no object is detected, search for it
                if (objectWidth == 0) { // No object detected
                    telemetry.addData("Status", "Searching for object.");
                    telemetry.update();
                    robot.setMotorPower(0.3, -0.3); // Turn slowly to search
                }

                // Sleep for a short time before the next loop iteration
                sleep(100); // Adjust sleep time as needed
            }

            // After object detection is done, start AprilTag detection
            aprilTagDetection.processDetectedTags(); // Process the tags detected by AprilTagDetection

            // Now get the detected tags from AprilTagDetection
            List<AprilTagDetection.TagDetection> aprilTags = aprilTagDetection.pipeline.getDetectedTags();

            if (aprilTags != null && !aprilTags.isEmpty()) {
                // Process the first detected AprilTag
                AprilTagDetection.TagDetection targetTag = aprilTags.get(0);
                telemetry.addData("AprilTag Detected", targetTag.getTagId());
                telemetry.update();

                // Process the detected tag's position and move towards it
                moveTowardsAprilTag(targetTag);
            } else {
                telemetry.addData("AprilTag Status", "No AprilTags detected");
                telemetry.update();
            }
        }
    }

    // Method to move robot towards the detected AprilTag
    private void moveTowardsAprilTag(AprilTagDetection.TagDetection targetTag) {
        // Get the corner coordinates of the AprilTag
        Point[] corners = targetTag.getCorners();
        Point topLeft = corners[0];
        Point topRight = corners[1];
        Point bottomLeft = corners[2];
        Point bottomRight = corners[3];

        // Calculate the center of the tag
        double tagCenterX = (topLeft.x + topRight.x + bottomLeft.x + bottomRight.x) / 4;
        double tagCenterY = (topLeft.y + topRight.y + bottomLeft.y + bottomRight.y) / 4;

        // Calculate distance and direction to the AprilTag
        double distanceToTag = aprilTagDetection.calculateDistance(topLeft, bottomRight);
        double imageCenterX = 320; // Assuming camera resolution is 640x480
        double imageCenterY = 240;

        // Determine if the tag is to the left or right, and move accordingly
        if (tagCenterX < imageCenterX - 50) {
            robot.setMotorPower(-0.3, 0.3); // Turn left (if tag is left)
            telemetry.addData("Turning", "Left");
        } else if (tagCenterX > imageCenterX + 50) {
            robot.setMotorPower(0.3, -0.3); // Turn right (if tag is right)
            telemetry.addData("Turning", "Right");
        } else {
            robot.setMotorPower(0.5, 0.5); // Move forward (if tag is centered)
            telemetry.addData("Moving", "Forward");
        }

        // If the tag is detected within a certain distance, stop
        if (distanceToTag < 10) {
            robot.stopMotors();
            telemetry.addData("Arrived", "At the AprilTag!");
            telemetry.update();
        }
    }
}
