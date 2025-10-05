package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@TeleOp(name="AprilTag")

public class Test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(1430,1430,480,620)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        VisionPortal visionPortal= new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam"))
                .setCameraResolution(new Size(640,480))
                .build();

        waitForStart();

        while (!isStopRequested()&& opModeIsActive()) {
             if(!tagProcessor.getDetections().isEmpty()){
                 AprilTagDetection tag = tagProcessor.getDetections().get(0);
                 telemetry.addData("x",tag.ftcPose.x);
                 telemetry.addData("y",tag.ftcPose.y);
                 telemetry.addData("roll",tag.ftcPose.roll);
                 telemetry.addData("pitch",tag.ftcPose.pitch);
                 telemetry.addData("yaw",tag.ftcPose.yaw);
                 telemetry.addData("TagId", tag.id);
             }

             telemetry.update();


         }
        }
    }
