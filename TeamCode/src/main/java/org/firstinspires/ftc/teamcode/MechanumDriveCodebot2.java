package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mechanum DriveCodebot2")
public class MechanumDriveCodebot2 extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    private CRServo intakeservoleft, intakeservoright;

    private DcMotor rshotMotor,lshotMotor;

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

    public void intake() {
        if (gamepad2.rightBumperWasPressed()){
           intakeservoright.setPower(0.5);
           intakeservoleft.setPower(-0.5);
        }
        if(gamepad2.leftBumperWasPressed()) {
            intakeservoleft.setPower(-0.5);
            intakeservoright.setPower(0.5);
        }
        if (gamepad2.y){
            intakeservoright.setPower(0);
            intakeservoleft.setPower(0);
        }
    }

    public void outake() {
        if (gamepad1.a){
            rshotMotor.setPower(0.35);
            lshotMotor.setPower(-0.35);
        }
        if(gamepad2.b) {
            rshotMotor.setPower(-.01);
            lshotMotor.setPower(0.01);
        }
        if (gamepad2.x){
            rshotMotor.setPower(0);
            lshotMotor.setPower(-0);
        }
    }







    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        intakeservoright = hardwareMap.get(CRServo.class,"rightservo");
        intakeservoleft = hardwareMap.get(CRServo.class,"leftservo");
        rshotMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        lshotMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");


        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
            this.intake();
            telemetry.update();
        }
    }
}
