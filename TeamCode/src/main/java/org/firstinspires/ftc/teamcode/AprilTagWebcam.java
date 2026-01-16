package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }
    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if(detectedId == null) {return;}
        if(detectedId.metadata != null) {
            telemetry.addLine(detectedId + ", " + detectedId.metadata.name);
            telemetry.addLine("XYZ: " + detectedId.ftcPose.x + ", " + detectedId.ftcPose.y + ", " + detectedId.ftcPose.z);
            telemetry.addLine("PRY: " + detectedId.ftcPose.pitch + ", " + detectedId.ftcPose.roll + ", " + detectedId.ftcPose.yaw);
            telemetry.addLine("RBE: " + detectedId.ftcPose.range + ", " + detectedId.ftcPose.bearing + ", " + detectedId.ftcPose.elevation);
        }
    }

    public void stop() {
        if(visionPortal != null) {
            visionPortal.close();
        }
    }

}

