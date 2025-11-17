package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorTest extends OpMode {
    ColorSensor color = new ColorSensor();
    @Override
    public void init() {
        color.init(hardwareMap);
    }

    @Override
    public void loop() {
        color.getDetectedColor(telemetry);
    }
}