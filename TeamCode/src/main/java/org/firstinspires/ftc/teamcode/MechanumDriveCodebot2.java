package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp (name="Slide Move by Inches")

public class MechanumDriveCodebot2 extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
        private CRServo intakeservoleft, intakeservoright;
    private DcMotor slideMotor1 = null;
    private DcMotor slideMotor2 = null;


    private ElapsedTime runtime = new ElapsedTime(); // <-- Add this line

        private DcMotor rshotMotor,lshotMotor;
        static final double COUNTS_PER_MOTOR_REV = 1440; // Example: AndyMark NeveRest 60 motor
        static final double DRIVE_GEAR_REDUCTION = 1.0;  // This is the gear ratio
        static final double SPOOL_DIAMETER_INCHES = 1.5; // Diameter of the spool in inches
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * Math.PI);
        static final double SLIDE_POWER = 0.5;



    @Override
    public void runOpMode() {


        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");

        slideMotor1 = hardwareMap.get(DcMotorEx.class, "slide_motor");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "slide_motor");


        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        moveSlideInches(10, SLIDE_POWER, 5.0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    public void drive() {
        //get joystick values
        double yPower = 0.7 * gamepad1.left_stick_y;   // forward and back
        double yaw = 0.7 * -gamepad1.right_stick_x;  // turning
        double xPower = 0.7 * gamepad1.left_stick_x;   // drive

        //calculate powers
        double flPower = yPower + xPower + yaw ;
        double frPower = yPower -xPower - yaw ;
        double blPower = yPower  -xPower + yaw;
        double brPower = yPower +xPower - yaw;


        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        if(maxPower > 1.0) {
            flPower /= maxPower;
            blPower /= maxPower;
            frPower /= maxPower;
            brPower /= maxPower;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);

        telemetry.addData("Motor power is set","%.3f %.3f %.3f %.3f ", flPower,frPower,blPower,brPower);
        telemetry.update();

    }
    public void moveSlideInches(double inches, double power, double timeoutS) {
        int newTargetPosition;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Calculate the new target position
            newTargetPosition = slideMotor1.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            slideMotor1.setTargetPosition(newTargetPosition);

            newTargetPosition = slideMotor2.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            slideMotor2.setTargetPosition(newTargetPosition);


            // Turn On RUN_TO_POSITION
            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start the motion.
            runtime.reset();
            slideMotor1.setPower(Math.abs(power));
            slideMotor2.setPower(Math.abs(power));

            // Keep looping while not at position, and the opmode is active, and the timeout has not been reached
            while (opModeIsActive() && runtime.seconds() < timeoutS && (slideMotor1.isBusy()) &&(slideMotor2.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTargetPosition);
                telemetry.addData("Path2", "Running at %7d", slideMotor1.getCurrentPosition());
                telemetry.addData("Path2", "Running at %7d", slideMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);

            // Turn off RUN_TO_POSITION mode
            slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            this.drive();
            this.moveSlideInches(9.2,0.5,5);

        }
    }
}
