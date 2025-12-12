package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LaunchZoneRed.TICKS_PER_REV;

import static java.util.Collections.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mechanum DriveCode")
public class MechanumDriveCode extends LinearOpMode {

    private boolean lockMotor = true;
    private boolean Buttonpressrise = false;
    private boolean Buttonpresslower = false;
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    static final double WHEEL_DIAMETER_INCHES = 1.504; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private DcMotor outakeMotor, slideMotor1,slideMotor2;
    private Servo artifactGate;

    public void drive() {
        //get joystick values
        double yPower = 0.7 * gamepad1.left_stick_y;   // forward and back
        double yaw = 0.7 * gamepad1.right_stick_x;  // turning
        double xPower = 0.7 * gamepad1.left_stick_x;   // drive

        //calculate powers
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

        telemetry.addData("Motor power is set", "%.3f %.3f %.3f %.3f ", flPower, frPower, blPower, brPower);
        telemetry.update();

    }

    public void outake() {
        if (gamepad2.a){
            outakeMotor.setPower(0.5);
        }
        if(gamepad2.b) {
            outakeMotor.setPower(-.01);
        }
        if (gamepad2.x){
            outakeMotor.setPower(0);
        }
    }

    public void release() {

        if (gamepad2.right_bumper) {
            artifactGate.setPosition(0.6);
        }
        if(gamepad2.left_bumper) {
            artifactGate.setPosition(1.0);
        }
    }

    public void unStuckBall(){
        if (gamepad2.y){
            outakeMotor.setPower(1);
            sleep(2000);
            outakeMotor.setPower(0);
        }
    }

    public void park(double inches,double speed) {


        int moveCounts = (int) (inches * COUNTS_PER_INCH);


        int slideMotor1Target = slideMotor1.getCurrentPosition() - moveCounts;
        int slideMotor2Target = slideMotor2.getCurrentPosition() - moveCounts;

        if (gamepad1.a && Buttonpressrise==false ){

            slideMotor2.setTargetPosition(slideMotor2Target);
            slideMotor1.setTargetPosition(slideMotor1Target);

            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor1.setPower(speed);
            slideMotor2.setPower(speed);

        }

       /* if (gamepad1.b && Buttonpresslower == false){

            slideMotor2.setTargetPosition(0);
            slideMotor1.setTargetPosition(0);

            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Motor Position", slideMotor2.getCurrentPosition());
            telemetry.update();
            slideMotor1.setPower(speed);
            slideMotor2.setPower(speed);


           Buttonpresslower = true;
        }

           */


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
        outakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
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
            this.outake();
            this.release();
            this.unStuckBall();
            this.park(8, 1);
            this.logData();
            telemetry.update();
        }
    }
}