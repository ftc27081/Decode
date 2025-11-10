package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "Obelisk Pattern Detector", group = "FTC DECODE")
public class ObeliskPatternDetector extends LinearOpMode {

    // --- SEGMENT 1: Declarations and Variables ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredDetection = null;
    private String detectedPattern = "Unknown";
    // Define the AprilTag IDs used for the DECODE obelisk (IDs are examples, verify with official manual)
    private static final int TAG_ID_GPP = 21; // Example: Green, Purple, Purple motif
    private static final int TAG_ID_PGP = 22; // Example: Purple, Green, Purple motif
    private static final int TAG_ID_PPG = 23; // Example: Purple, Purple, Green motif

    @Override
    public void runOpMode() throws InterruptedException {

        initAprilTag();
        telemetry.addData("Status", "AprilTag Initialized. Ready to run.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Continuously check for the tag until it's found or the autonomous period is over
            while (opModeIsActive() && detectedPattern.equals("Unknown")) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) { // Check if the detection has valid metadata
                        int tagId = detection.id;
                        if (tagId == TAG_ID_GPP) {
                            detectedPattern = "GPP";
                            telemetry.addData("Pattern is GPP",tagId );

                            break; // Found the pattern, exit the for loop
                        } else if (tagId == TAG_ID_PGP) {
                            detectedPattern = "PGP";
                            telemetry.addData("Pattern is PGP",tagId );
                            break;
                        } else if (tagId == TAG_ID_PPG) {
                            detectedPattern = "PPG";
                            telemetry.addData("Pattern is PPG",tagId );

                            break;
                        }
                    }
                }

                // Update telemetry with the current status
                telemetry.addData("Detected Pattern", detectedPattern);
                telemetry.update();

                sleep(20); // Small delay to prevent CPU overuse
            }


            // Once the pattern is detected, the robot can act on it
            telemetry.addData("Final Pattern Detected", detectedPattern);
            telemetry.update();

            // Here you would add your robot's movement code based on the detectedPattern variable
            if (detectedPattern.equals("GPP")) {
                // Code to drive to the GPP scoring location/execute GPP strategy
            } else if (detectedPattern.equals("PGP")) {
                // Code to drive to the PGP scoring location/execute PGP strategy
            } else if (detectedPattern.equals("PPG")) {
                // Code to drive to the PPG scoring location/execute PPG strategy
            }

            // Keep the OpMode active until STOP is pressed for autonomous period completion
            while(opModeIsActive()){
                sleep(20);
            }
        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        // The camera name ("Webcam 1") should match the configuration on the Control Hub.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
