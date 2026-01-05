package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LaunchZoneRed.TICKS_PER_REV;
import static java.util.Collections.max;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Mechanum DriveCode")
public class MechanumDriveCode extends LinearOpMode {


    double lastError = 0;
    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();
    public int runMotor = 0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double targetvalue = 10;

    private DcMotorEx flMotor, frMotor, blMotor, brMotor, outakeMotor;
    static final double WHEEL_DIAMETER_INCHES = 1.504; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private DcMotor slideMotor1,slideMotor2;
    private Servo artifactGate;

    public void drive() {
        //get joystick values
        double yPower = 0.7 * gamepad1.left_stick_y;   // forward and back
        double yaw = 0.7 * gamepad1.right_stick_x;  // turning
        double xPower = 0.7 * gamepad1.left_stick_x;   // drive

        //calculate powersf
        double frPower = yPower + xPower + yaw;
        double flPower = yPower - xPower - yaw;
        double brPower = yPower - xPower + yaw;
        double blPower = yPower + xPower - yaw;

        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            blPower /= maxPower;
            frPower /= maxPower;
            brPower /= maxPower;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);


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
        integralSum += error * dt;
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



    public void powerControl() {
        if (gamepad2.a) {
           runMotor = 1;
        }

        if (gamepad2.x){
            outakeMotor.setPower(-.01);
            sleep(2000);
            outakeMotor.setPower(0);
            runMotor = 0;
        }
    }

    public void release() {

        if (gamepad2.right_bumper) {
            artifactGate.setPosition(0.5);
        }
        if(gamepad2.left_bumper) {
            artifactGate.setPosition(1.0);
        }
    }

    public void unStuckBall(){
        if (gamepad2.y){
            outakeMotor.setPower(1);
        }
    }


    public void logData() {
        if(gamepad1.b) {
            telemetry.addData("left motor:", slideMotor1.getCurrentPosition());
            telemetry.addData("right motor:", slideMotor2.getCurrentPosition());
        }
    }


    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        outakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        slideMotor1 = hardwareMap.get(DcMotor.class, "leftMotor");
        slideMotor2 = hardwareMap.get(DcMotor.class, "rightMotor");

        artifactGate = hardwareMap.get(Servo.class,"artifactGate");
        artifactGate.setDirection(Servo.Direction.FORWARD);
        artifactGate.setPosition(1.0);

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor1.setTargetPosition(slideMotor1.getCurrentPosition()-1);
        slideMotor2.setTargetPosition(slideMotor2.getCurrentPosition()-1);

        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor1.setPower(-0.1);
        slideMotor2.setPower(-0.1);
        outakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());


    }

    public void runOpMode() {
        // init
        initialization();

        // create multiple telemetries and add to dashboard
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            this.drive();
            this.powerControl();
            this.release();
            this.unStuckBall();
            this.logData();
            if(runMotor==1) {
                this.wheelVelocity(outakeMotor,targetvalue);
            }
            telemetry.update();
        }
    }
}