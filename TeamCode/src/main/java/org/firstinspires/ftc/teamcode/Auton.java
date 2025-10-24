package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name = "Basic Auton", group = "Autonomous")
public class Auton extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private ElapsedTime runtime = new ElapsedTime();

    // Motor ticks per revolution
    static final double TICKS_PER_REV = 537.6;  // For 312 RPM Yellow Jackets
    static final double WHEEL_DIAMETER_INCHES = 4.0; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        // Map motors
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        lb = hardwareMap.get(DcMotor.class, "leftRear");
        rb = hardwareMap.get(DcMotor.class, "rightRear");

        // Set motor direction
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(0.5, 24, 5); // Drive forward 24 inches at 50% power, 5 second timeout
            encoderDrive(0.5, -30, 5);
            encoderStrafe(0.5, 15, 5);
            encoderStrafe(0.5, -20, 5);


        }
    }

    private void resetEncoders() {
        DcMotor[] motors = {lf, rf, lb, rb};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    private void encoderStrafe(double speed, double inches, double timeoutS) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        // Strafing adjustments for each motor
        int leftFrontTarget = lf.getCurrentPosition() + moveCounts;
        int rightFrontTarget = rf.getCurrentPosition() - moveCounts;
        int leftRearTarget = lb.getCurrentPosition() - moveCounts;
        int rightRearTarget = rb.getCurrentPosition() + moveCounts;

        // Set Target Positions
        lf.setTargetPosition(leftFrontTarget);
        rf.setTargetPosition(rightFrontTarget);
        lb.setTargetPosition(leftRearTarget);
        rb.setTargetPosition(rightRearTarget);

        // Switch to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {
            telemetry.addData("Strafing", "Running at %.2f inches", inches);
            telemetry.update();
        }

        // Stop all motion
        stopAllMotors();

        // Reset to RUN_USING_ENCODER
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderDrive(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget = lf.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightFrontTarget = rf.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLeftRearTarget = lb.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightRearTarget = rb.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        // Set Target Positions
        lf.setTargetPosition(newLeftFrontTarget);
        rf.setTargetPosition(newRightFrontTarget);
        lb.setTargetPosition(newLeftRearTarget);
        rb.setTargetPosition(newRightRearTarget);

        // Switch to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        // Keep looping while we are still active, and all motors are running to position
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {
            telemetry.addData("Path", "Driving to %.2f inches", inches);
            telemetry.update();
        }

        // Stop all motion
        stopAllMotors();

        // Reset to RUN_USING_ENCODER
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAllMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
