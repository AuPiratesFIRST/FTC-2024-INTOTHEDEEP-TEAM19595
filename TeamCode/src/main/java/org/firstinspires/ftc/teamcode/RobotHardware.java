package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    private DcMotor backLeftMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;

    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_BASE = 16.0;    // inches between wheels

    private Telemetry telemetry;
    private double wheelCircumference;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeMotors(hardwareMap);
        wheelCircumference = Math.PI * WHEEL_DIAMETER;
    }

    private void initializeMotors(HardwareMap hardwareMap) {
        try {
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);

            resetEncoders();
        } catch (Exception e) {
            telemetry.addData("Error", "Motor initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    public void forwardForDistance(double inches, double power) {
        resetEncoders();
        int ticks = calculateTicks(inches);
        setTargetPositions(ticks);
        runMotorsToPosition(power);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 10) {
            double remaining = Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition());
            double adjustedPower = Math.max(0.2, power * (remaining / ticks));

            setMotorPowers(adjustedPower);
            displayTelemetry("Moving Forward", inches, ticks);
        }

        stopMotors();
    }

    public void turn(int degrees, boolean clockwise) {
        double turnCircumference = Math.PI * WHEEL_BASE;
        double ticksPerDegree = (TICKS_PER_REVOLUTION / wheelCircumference) * turnCircumference / 360.0;
        int ticks = (int) (ticksPerDegree * degrees);

        if (clockwise) {
            setTargetPositionsForTurn(ticks, -ticks);
        } else {
            setTargetPositionsForTurn(-ticks, ticks);
        }

        runMotorsToPosition(0.5);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 5) {
            double remaining = Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition());
            double power = Math.max(0.2, 0.5 * (remaining / ticks));

            frontLeftMotor.setPower(clockwise ? power : -power);
            backLeftMotor.setPower(clockwise ? power : -power);
            frontRightMotor.setPower(clockwise ? -power : power);
            backRightMotor.setPower(clockwise ? -power : power);

            displayTelemetry("Turning", degrees, ticks);
        }

        stopMotors();
    }

    public void rotateClockwise(int degrees, double power) {
    rotate(degrees, power, true);
}

public void rotateCounterClockwise(int degrees, double power) {
    rotate(degrees, power, false);
}

private void rotate(int degrees, double power, boolean clockwise) {
    resetEncoders();

    double turnCircumference = Math.PI * WHEEL_BASE;
    double ticksPerDegree = (TICKS_PER_REVOLUTION / wheelCircumference) * turnCircumference / 360.0;
    int ticks = (int) (ticksPerDegree * degrees);

    if (clockwise) {
        setTargetPositionsForTurn(ticks, -ticks);
    } else {
        setTargetPositionsForTurn(-ticks, ticks);
    }

    runMotorsToPosition(power);

    ElapsedTime runtime = new ElapsedTime();
    while (motorsBusy() && runtime.seconds() < 5) {
        double remaining = Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition());
        double adjustedPower = Math.max(0.2, power * (remaining / ticks));

        frontLeftMotor.setPower(clockwise ? adjustedPower : -adjustedPower);
        backLeftMotor.setPower(clockwise ? adjustedPower : -adjustedPower);
        frontRightMotor.setPower(clockwise ? -adjustedPower : adjustedPower);
        backRightMotor.setPower(clockwise ? -adjustedPower : adjustedPower);

        displayTelemetry(clockwise ? "Rotating Clockwise" : "Rotating Counterclockwise", degrees, ticks);
    }

    stopMotors();
}

    public void strafe(double inches, boolean right) {
        resetEncoders();
        int ticks = calculateTicks(inches);

        if (right) {
            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + ticks);
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - ticks);
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - ticks);
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + ticks);
        } else {
            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - ticks);
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + ticks);
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + ticks);
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - ticks);
        }

        runMotorsToPosition(0.5);

        while (motorsBusy()) {
            displayTelemetry("Strafing", inches, ticks);
        }

        stopMotors();
    }

    private int calculateTicks(double inches) {
        double rotations = inches / wheelCircumference;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    private void setTargetPositions(int ticks) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + ticks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + ticks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + ticks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + ticks);
    }

    private void setTargetPositionsForTurn(int leftTicks, int rightTicks) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + leftTicks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + leftTicks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + rightTicks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + rightTicks);
    }

    private void runMotorsToPosition(double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPowers(power);
    }

    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() || backLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy();
    }

    private void setMotorPowers(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void stopMotors() {
        setMotorPowers(0);
    }

    private void displayTelemetry(String action, double distanceOrAngle, int ticks) {
        telemetry.addData(action, "Target: %.2f, Ticks: %d", distanceOrAngle, ticks);
        telemetry.addData("FL", "Pos: %d, Pow: %.2f", frontLeftMotor.getCurrentPosition(), frontLeftMotor.getPower());
        telemetry.addData("FR", "Pos: %d, Pow: %.2f", frontRightMotor.getCurrentPosition(), frontRightMotor.getPower());
        telemetry.addData("BL", "Pos: %d, Pow: %.2f", backLeftMotor.getCurrentPosition(), backLeftMotor.getPower());
        telemetry.addData("BR", "Pos: %d, Pow: %.2f", backRightMotor.getCurrentPosition(), backRightMotor.getPower());
        telemetry.update();
    }
}
