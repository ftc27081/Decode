package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="outaketest")
public class outaketest extends LinearOpMode {
    private DcMotorEx outakemotor,intakeleft,intakeright;

    public void outake() {

        if (gamepad1.a){
            outakemotor.setPower(0.5);
        }

        if (gamepad1.b){

            outakemotor.setPower(-.675);
        }

        if (gamepad1.x){
            outakemotor.setPower(0);
        }

    }


    public void intaketest (){

        if(gamepad2.a){

            intakeleft.setPower(1);
            intakeright.setPower(-1);


        }


        if(gamepad2.b){

            intakeleft.setPower(-1 );
            intakeright.setPower(1);
        }


        if (gamepad1.x){
            intakeright.setPower(0);
            intakeleft.setPower(0);
        }



    }
    public void initialization() {
        // Initialization
        outakemotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        intakeright = hardwareMap.get(DcMotorEx.class, "brMotor");
        intakeleft = hardwareMap.get(DcMotorEx.class, "blMotor");


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
            this.intaketest();
            telemetry.update();
        }
    }
}