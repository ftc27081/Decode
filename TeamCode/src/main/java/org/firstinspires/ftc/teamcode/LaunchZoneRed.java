package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MechanumDriveCode.f;
import static org.firstinspires.ftc.teamcode.MechanumDriveCode.p;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Launch Zone Red ", group = "Autonomous")
public class LaunchZoneRed extends LinearOpMode {

    private DcMotorEx flMotor, frMotor, blMotor, brMotor,outakeMotor;
    private Servo artifactGate;
    double lastError = 0;

    private int turnonshooter =0;
    ElapsedTime timer = new ElapsedTime();
    public static double p = 0.002;
    public static double i = 0;
    public static double d = 0.000102;
    public static double f = 0.00043;
    boolean gateNotOpen = true;
    public static double targetValue = 5800;

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
        outakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        artifactGate = hardwareMap.get(Servo.class,"artifactGate");
        artifactGate.setDirection(Servo.Direction.FORWARD);
        artifactGate.setPosition(1.0);

        // Set motor direction
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set motor direction
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        waitForStart();
        ElapsedTime gateControl= new ElapsedTime();

        if (opModeIsActive()) {
            encoderDrive(0.25,  1, 5); // Drive backward 26 inches at 50% power, 5 second timeout
            while(turnonshooter == 1){
                wheelVelocity(outakeMotor,targetValue);
                if(gateControl.seconds() > 5 && gateNotOpen) {
                    openGate();
                }
                if(gateControl.seconds() > 9) {
                    closeGate();
                    turnonshooter = 0;
                }
            }
            stopShooterMotor();
            encoderDrive(0.25,  15, 5); // move out of zone after shooting
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

        turnonshooter=1;
    }




    public void wheelVelocity(DcMotorEx motor, double targetVelocity) {

        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        double dt = timer.seconds();
        timer.reset();

        telemetry.addData("Seconds passed", dt);
        telemetry.addData("milisceonds passed",dt*0.0001);
        telemetry.update();

        // Protect against divide-by-zero
        if (dt <= 0) return;

        // ----- FEEDFORWARD -----
        double fComponent = f * targetVelocity;

        // ----- PROPORTIONAL -----
        double pComponent = p * error;

        // ----- INTEGRAL -----
        double integralSum = error * dt;
        double iComponent = i * integralSum;

        // ----- DERIVATIVE -----
        double derivative = (error - lastError) / dt;
        double dComponent = d * derivative;

        // ----- TOTAL POWER -----
        double power = fComponent + pComponent + iComponent + dComponent;

        // Clamp motor power
        power = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(power);

        lastError = error;

        // Telemetry
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("P", pComponent);
        telemetry.addData("I", iComponent);
        telemetry.addData("D", dComponent);
        telemetry.addData("F", fComponent);
    }

    private void openGate(){
        artifactGate.setPosition(0.5);
        gateNotOpen = false;
    }

    private void closeGate() {
        artifactGate.setPosition(1);
    }

    private void stopShooterMotor() { outakeMotor.setPower(0); }

    private void stopAllMotors() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}