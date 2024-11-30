package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoWinFull", group = "Competition")
public class AutoWinFull extends LinearOpMode {

    // Declare Robot Hardware and Arm Movement
    private RobotHardware robotHardware;
    private ArmMovement armMovement;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        if (opModeIsActive()) {
            try {
                telemetry.addData("Step 1", "Moving to Target Zone...");
                telemetry.update();
                moveToTargetZone(36); // Move forward 36 inches

                telemetry.addData("Opening Gripper");
                telemetry.update();
                armMovement.openGripper();
                sleep(1000); // Wait for 1 second

                telemetry.addData("Moving Arm");
                telemetry.update();
                armMovement.move(10); // Move arm 10 inches forward
                sleep(500); // Wait for movement to complete

                telemetry.addData(, "Closing Gripper");
                telemetry.update();
                armMovement.closeGripper();
                sleep(1000);

                telemetry.addData("Parking in Safe Zone");
                telemetry.update();
                moveToTargetZone(-24); // Move backward 24 inches

                // Final step: Stop all motors
                telemetry.addData("Autonomous Complete");
                telemetry.update();
                stopMotors();
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
            }
        }
    }

    /**
     * Moves the robot a specific distance in inches.
     * @param inches Distance to move forward (positive) or backward (negative).
     */
    private void moveToTargetZone(double inches) {
        telemetry.addData("Moving", inches + " inches");
        telemetry.update();
        robotHardware.forwardForDistance(inches);
    }

    /**
     * Stops all motors for safety.
     */
    private void stopMotors() {
        robotHardware.stopMotors();
    }
}
