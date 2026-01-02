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


    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;
    public int runMotor = 0;
    public static double p = 0.001, i = 0.001, d = 0.0001, f = 0.00036;
    public static double targetvalue = 1250;
    private boolean lockMotor = true;
    private boolean Buttonpressrise = false;
    private boolean Buttonpresslower = false;
    private DcMotorEx flMotor, frMotor, blMotor, brMotor, outakeMotor;
    static final double WHEEL_DIAMETER_INCHES = 1.504; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private DcMotor slideMotor1,slideMotor2;
    private Servo artifactGate;
    static double omVelocity;
// git test
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

     if (gamepad2.a) {
         sleep(2000);
         timer.reset();
         double milliseconds = timer.milliseconds();
         timer.reset();
         double power;

         double currentVelocity = motor.getVelocity();
         double error = targetVelocity - currentVelocity;
         //calculate p component
         double pComponent = error * p;

         //calculate i component
         double integralSum = error * milliseconds;
         double iComponent = integralSum * i;

         //calculate d component
         double dComponent = (error - lastError) / milliseconds;

         power = pComponent + iComponent + dComponent;
         outakeMotor.setPower(power);
         lastError = error;


         telemetry.addData("p", p);
         telemetry.addData("i", i);
         telemetry.addData("d", d);
         telemetry.addData("feed forword", f);
         telemetry.addData("targetVelocity", targetvalue);
         telemetry.addData("Current Velocity", outakeMotor.getVelocity());
         telemetry.addData("p component",pComponent);
         telemetry.addData("i component",iComponent);
         telemetry.addData("d component",dComponent);


     }
    }



    public void powerControl() {
        if (gamepad2.x){
            outakeMotor.setPower(-.01);
            sleep(2000);
            outakeMotor.setPower(0);
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
            this.drive();// drive
            this.powerControl(); // stop and set power for shooter
            this.release();      // open and close gate
            this.unStuckBall();  // unstuck ball
            this.logData();      // adds data to telemetry
            this.wheelVelocity(outakeMotor,targetvalue); // reset the lost power
            telemetry.update();
        }
    }
}