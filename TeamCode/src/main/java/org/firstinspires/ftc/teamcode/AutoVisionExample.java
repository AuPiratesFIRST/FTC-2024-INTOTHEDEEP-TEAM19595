import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.vision.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.vision.VisionProcessor;

@Autonomous(name="Auto Vision Example", group="Autonomous")
public class AutoVisionExample extends OpMode {

    // Vision processing objects
    private VisionPortal myVisionPortal;
    private AprilTagProcessor myAprilTagProcessor;
    private TfodProcessor myTfodProcessor;

    // Robot hardware variables
    private DcMotor leftDrive, rightDrive;
    private Servo claw;

    @Override
    public void init() {
        // Initialize hardware (motors and servos)
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        claw = hardwareMap.get(Servo.class, "claw");

        // Initialize vision systems
        initVision();
    }

    private void initVision() {
        // Setup the vision portal and processors (only using one camera)
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        // Initialize the AprilTag processor
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        
        // Initialize the TFOD processor
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();

        // Create and initialize the vision portal (handles both processors)
        myVisionPortal = new VisionPortal.Builder()
            .addProcessor(myAprilTagProcessor)
            .addProcessor(myTfodProcessor)
            .build();

        // Enable the vision portal
        myVisionPortal.enable();
    }

    @Override
    public void start() {
        // Start vision processing
        myVisionPortal.start();
        
        // Begin autonomous actions
        runAutonomous();
    }

    private void runAutonomous() {
        // Autonomous Logic Example:
        // 1. Use AprilTag processor for navigation.
        // 2. Use TFOD processor to detect objects (cargo).
        
        // Wait for vision data (AprilTag and TFOD results)
        AprilTagProcessor.DetectionData aprilTagData = myAprilTagProcessor.getLastDetectedTags();
        TfodProcessor.Recognition tfodData = myTfodProcessor.getRecognitions().get(0);  // Assuming first detected item is the target

        // Example navigation based on AprilTag data
        if (aprilTagData != null && !aprilTagData.isEmpty()) {
            // Use the AprilTag data for positioning (e.g., determine position relative to the shipping hub)
            double tagPosition = aprilTagData.get(0).getPosition().x;  // Example position data (X-coordinate)
            
            // Navigate based on position
            if (tagPosition < 0.5) {
                moveForward();  // Move the robot forward
            } else {
                turnLeft();  // Turn if necessary
            }
        }
        
        // Example cargo detection with TFOD
        if (tfodData != null) {
            // Check for detected object (e.g., cargo)
            String label = tfodData.getLabel();
            if ("Cone".equals(label)) {
                grabCargo();  // Grab the detected cargo
            }
        }
        
        // Example parking logic (align to charge station)
        parkOnChargeStation();
    }

    private void moveForward() {
        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);
    }

    private void turnLeft() {
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
    }

    private void grabCargo() {
        claw.setPosition(0.5);  // Grab the cargo
    }

    private void parkOnChargeStation() {
        // Use AprilTag for precise parking alignment
        double position = myAprilTagProcessor.getLastDetectedTags().get(0).getPosition().x;
        if (position > 0.5) {
            // Adjust robot position to park (if necessary)
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);
        }
    }

    @Override
    public void loop() {
        // Display the latest vision data (for debugging purposes)
        telemetry.addData("AprilTag Position", myAprilTagProcessor.getLastDetectedTags());
        telemetry.addData("TFOD Object", myTfodProcessor.getRecognitions());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop vision processing
        myVisionPortal.stop();
    }
}
