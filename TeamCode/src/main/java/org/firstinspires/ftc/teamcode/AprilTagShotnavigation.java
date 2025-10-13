package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = " AprilTag Drive ", group = "Autonomous")
public class AprilTagShotnavigation extends LinearOpMode {

    // Drive motors
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    // Vision-related variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TARGET_APRILTAG_ID = 20;
    private static final double DESIRED_DISTANCE = 25;


    private static final double DISTANCE_TOLERANCE = 1.0;
    private static final double BEARING_TOLERANCE = 1.0;


    private static final double TICKS_PER_INCH = 537.6 ;

    private static final double DRIVE_POWER = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized. Point camera at AprilTag ID %d", TARGET_APRILTAG_ID);
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            driveToAprilTag();
        }
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        visionPortal.close();
    }


    private void driveToAprilTag() {
        while (opModeIsActive()) {
            AprilTagDetection detection = findTargetAprilTag(TARGET_APRILTAG_ID);

            if (detection != null) {
                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;
                double yaw = detection.ftcPose.yaw;

                // Check if we are close enough to stop
                if (Math.abs(range - DESIRED_DISTANCE) <= DISTANCE_TOLERANCE && Math.abs(bearing) <= BEARING_TOLERANCE) {
                    telemetry.addData("Status", "Target reached!");
                    telemetry.update();
                    stopAndResetMotors();
                    break;
                }

                // Calculate incremental movement based on current pose error
                double driveInches = (range - DESIRED_DISTANCE);
                double strafeInches = bearing; // Bearing is an angle, but we can treat it proportionally for strafing
                double turnInches = yaw; // Yaw is also an angle

                // Convert inches to motor ticks
                int driveTicks = (int) (driveInches * TICKS_PER_INCH);
                int strafeTicks = (int) (strafeInches * TICKS_PER_INCH); // This is a rough approximation
                int turnTicks = (int) (turnInches * TICKS_PER_INCH); // This is a rough approximation

                // Set target positions based on incremental movement
                setTargetPositionForMecanum(driveTicks, strafeTicks, turnTicks);

                // Telemetry for monitoring
                telemetry.addData("Status", "Approaching Tag ID: %d", TARGET_APRILTAG_ID);
                telemetry.addData("Range", "%.2f", range);
                telemetry.addData("Bearing", "%.2f", bearing);
                telemetry.addData("Yaw", "%.2f", yaw);

                // Wait for the motors to reach their target positions before re-checking the tag
                while (opModeIsActive() && motorsAreBusy()) {
                    idle();
                }

            } else {
                telemetry.addData("Status", "Target Tag %d Not Found", TARGET_APRILTAG_ID);
                telemetry.addData("Action", "Searching...");
                stopAndResetMotors();
            }

            telemetry.update();
        }
    }

    private AprilTagDetection findTargetAprilTag(int tagId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    private void setTargetPositionForMecanum(int driveTicks, int strafeTicks, int turnTicks) {
        // Calculate new target positions
        int flTarget = flMotor.getCurrentPosition() + driveTicks + strafeTicks + turnTicks;
        int frTarget = frMotor.getCurrentPosition() + driveTicks - strafeTicks - turnTicks;
        int blTarget = blMotor.getCurrentPosition() + driveTicks - strafeTicks + turnTicks;
        int brTarget = brMotor.getCurrentPosition() + driveTicks + strafeTicks - turnTicks;

        // Set the new target positions
        flMotor.setTargetPosition(flTarget);
        frMotor.setTargetPosition(frTarget);
        blMotor.setTargetPosition(blTarget);
        brMotor.setTargetPosition(brTarget);

        // Set all motors to RUN_TO_POSITION mode
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors with a fixed power level
        flMotor.setPower(DRIVE_POWER);
        frMotor.setPower(DRIVE_POWER);
        blMotor.setPower(DRIVE_POWER);
        brMotor.setPower(DRIVE_POWER);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        flMotor.setMode(mode);
        frMotor.setMode(mode);
        blMotor.setMode(mode);
        brMotor.setMode(mode);
    }

    private void stopAndResetMotors() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean motorsAreBusy() {
        return flMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy();
    }
}
