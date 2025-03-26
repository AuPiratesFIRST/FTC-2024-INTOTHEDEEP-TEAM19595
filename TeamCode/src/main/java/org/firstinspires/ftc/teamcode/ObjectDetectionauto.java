package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Object Detection Example", group="Autonomous")
public class ObjectDetectionauto extends LinearOpMode {

    private ObjectDetection objectDetection;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the object detection system
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start streaming camera feed to dashboard
        dashboard.startCameraStream(objectDetection.getCamera(), 30); // Stream the camera from ObjectDetection
        waitForStart();

        while (opModeIsActive()) {
            // Process the camera frame and update the telemetry with object data
            double distance = objectDetection.objectDetectionPipeline.getDistance();
            double centroidX =  objectDetection.objectDetectionPipeline.getCentroidX();
            double centroidY = objectDetection.objectDetectionPipeline.getCentroidY();
            double objectWidth = objectDetection.objectDetectionPipeline.getObjectWidth();

            // Display data on the driver station
            telemetry.addData("Object Distance", distance);
            telemetry.addData("Object Centroid X", centroidX);
            telemetry.addData("Object Centroid Y", centroidY);
            telemetry.addData("Object Width", objectWidth);
            telemetry.update();

            sleep(100);  // Slow down the loop a bit to make it readable
        }
    }
}
