package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mechanum DriveCode")
public class MechanumDriveCode extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    private DcMotor outakeMotor;
    private Servo artifactGate;

    public void drive() {
        //get joystick values
        double xPower = 0.7 * gamepad1.right_stick_y;
        double yPower = 0.7 * gamepad1.left_stick_x;
        double yaw = 0.7 * gamepad1.left_stick_y;

        //calculate powers
        double flPower = xPower + yPower + yaw;
        double frPower = -xPower + yPower - yaw;
        double blPower = -xPower + yPower + yaw;
        double brPower = xPower + yPower - yaw;

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
    }

    public void outake() {
        if (gamepad2.a){
           outakeMotor.setPower(0.5);
        }
        if(gamepad2.b) {
            outakeMotor.setPower(-0.5);
        }
        if (gamepad2.x){
            outakeMotor.setPower(0);
        }
    }

    public void release() {

        if (gamepad2.right_bumper) {
            artifactGate.setPosition(0.25);
        }
        if(gamepad2.left_bumper) {
            artifactGate.setPosition(1.0);
        }
    }


    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        outakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        artifactGate = hardwareMap.get(Servo.class,"artifactGate");
        artifactGate.setDirection(Servo.Direction.FORWARD);
        artifactGate.setPosition(1.0);

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
            this.release();
            telemetry.update();
        }
    }
}
