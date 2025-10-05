package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="outaketest")
public class outaketest extends LinearOpMode {
    private DcMotorEx outakemotor;

    public void outake() {

        if (gamepad1.a){
            outakemotor.setPower(1);
        }

        if (gamepad1.b){

            outakemotor.setPower(-1);
        }

        if (gamepad1.x){
            outakemotor.setPower(0);
        }

    }

    public void initialization() {
        // Initialization
        outakemotor = hardwareMap.get(DcMotorEx.class, "fr");


    }

    public void runOpMode() {
        // init
        initialization();

        // create multiple telemetries and add to dashboard
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            this.outake();
            telemetry.update();
        }
    }
}