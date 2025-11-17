package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Depot Zone Blue", group = "Autonomous")
public class DepotZoneBlue extends LinearOpMode {

    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    private DcMotor outakeMotor;
    private Servo artifactGate;

    private ElapsedTime runtime = new ElapsedTime();

    // Motor ticks per revolution
    static final double TICKS_PER_REV = 537.6;  // For 312 RPM Yellow Jackets
    static final double WHEEL_DIAMETER_INCHES = 4.0; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        // Map motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        outakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        outakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        artifactGate = hardwareMap.get(Servo.class,"artifactGate");
        artifactGate.setDirection(Servo.Direction.FORWARD);
        artifactGate.setPosition(1.0);

        // Set motor direction
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(0.5,  20, 5); // Drive backward 26 inches at 50% power, 5 second timeout
            shootBalls();
            encoderStrafe(0.5,17,5); // move out of zone after shooting
        }
    }

    private void resetEncoders() {
        DcMotor[] motors = {flMotor, frMotor, blMotor, brMotor};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encoderStrafe(double speed, double inches, double timeoutS) {

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        // Strafing adjustments for each motor
        int leftFrontTarget = flMotor.getCurrentPosition() + moveCounts;
        int rightFrontTarget = frMotor.getCurrentPosition() - moveCounts;
        int leftRearTarget = blMotor.getCurrentPosition() - moveCounts;
        int rightRearTarget = brMotor.getCurrentPosition() + moveCounts;

        // Set Target Positions
        flMotor.setTargetPosition(leftFrontTarget);
        frMotor.setTargetPosition(rightFrontTarget);
        blMotor.setTargetPosition(leftRearTarget);
        brMotor.setTargetPosition(rightRearTarget);

        // Switch to RUN_TO_POSITION
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        flMotor.setPower(speed);
        frMotor.setPower(speed);
        blMotor.setPower(speed);
        brMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (flMotor.isBusy() && frMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy())) {
            telemetry.addData("Strafing", "Running at %.2f inches", inches);
            telemetry.update();
        }

        // Stop all motion
        stopAllMotors();

        // Reset to RUN_USING_ENCODER
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderDrive(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget = flMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightFrontTarget = frMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLeftRearTarget = blMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightRearTarget = brMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        // Set Target Positions
        flMotor.setTargetPosition(newLeftFrontTarget);
        frMotor.setTargetPosition(newRightFrontTarget);
        blMotor.setTargetPosition(newLeftRearTarget);
        brMotor.setTargetPosition(newRightRearTarget);

        // Switch to RUN_TO_POSITION
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        flMotor.setPower(speed);
        frMotor.setPower(speed);
        blMotor.setPower(speed);
        brMotor.setPower(speed);

        // Keep looping while we are still active, and all motors are running to position
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (flMotor.isBusy() && frMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy())) {
            telemetry.addData("Path", "Driving to %.2f inches", inches);
            telemetry.update();
        }

        // Stop all motion
        stopAllMotors();

        // Reset to RUN_USING_ENCODER
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderTurn(double speed, double degrees, double timeoutS) {

        double TURN_DIAMETER_INCHES = 18.0; // Distance between left and right wheels (adjust for your robot)
        double TURN_CIRCUMFERENCE = TURN_DIAMETER_INCHES * Math.PI;

        // Fraction of a full circle
        double turnFraction = Math.abs(degrees) / 360.0;

        // How far each wheel travels during the turn
        double turnDistance = TURN_CIRCUMFERENCE * turnFraction;

        // Convert to encoder counts
        int moveCounts = (int) (turnDistance * COUNTS_PER_INCH);

        // Left and right sides move in opposite directions
        int leftTargetChange = (degrees > 0) ? moveCounts : -moveCounts;
        int rightTargetChange = (degrees > 0) ? -moveCounts : moveCounts;

        // Calculate new targets
        int newLeftFrontTarget = flMotor.getCurrentPosition() + leftTargetChange;
        int newLeftRearTarget = blMotor.getCurrentPosition() + leftTargetChange;
        int newRightFrontTarget = frMotor.getCurrentPosition() + rightTargetChange;
        int newRightRearTarget = brMotor.getCurrentPosition() + rightTargetChange;

        // Set target positions
        flMotor.setTargetPosition(newLeftFrontTarget);
        blMotor.setTargetPosition(newLeftRearTarget);
        frMotor.setTargetPosition(newRightFrontTarget);
        brMotor.setTargetPosition(newRightRearTarget);

        // Set to RUN_TO_POSITION mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        flMotor.setPower(Math.abs(speed));
        blMotor.setPower(Math.abs(speed));
        frMotor.setPower(Math.abs(speed));
        brMotor.setPower(Math.abs(speed));

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (flMotor.isBusy() && frMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy())) {
            telemetry.addData("Turning", "%.1f degrees", degrees);
            telemetry.update();
        }


        // Stop all motion
        stopAllMotors();

        // Return to normal encoder mode
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void shootBalls(){
        outakeMotor.setPower(0.6);
        sleep(5000);
        //open the gate to launch the balls
        artifactGate.setPosition(0.25);
        sleep(8000);
        outakeMotor.setPower(0);
    }

    private void stopAllMotors() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}