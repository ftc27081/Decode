package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mechanum Drive Code PID")
public class MechanumDriveCode extends LinearOpMode {
    private DcMotorEx flMotor, frMotor, blMotor, brMotor;
    private DcMotorEx outakemotor;

    private CRServo release;
    public static double p = 0.003, i = 0.5, d = 1.5;
    public static double f = 0.001;
    public static double target = 0.0;


    // Drive code
    public void drive() {
        //get joystick values
        double xPower = 0.7 * gamepad1.left_stick_x;
        double yPower = 0.7 * -gamepad1.left_stick_y;
        double yaw = 0.7 * gamepad1.right_stick_x;

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


            outakemotor.setPower(gamepad1.left_stick_y);

    }

    public void release(){

        if(gamepad2.a){
            release.setPower(0.75);
        }



    }

    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        outakemotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        release = hardwareMap.get(CRServo.class,"Servo");


        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse direction of motors
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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