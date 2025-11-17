package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public void init (HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, ""); //Put name of sensor in control hub in quotations

    }
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //will return the 4 earlier values

        float normGREEN, normPURPLE;
        normGREEN = colors.green / colors.alpha;
        normPURPLE = colors.red + colors.blue / colors.alpha;

        telemetry.addData("Green", normGREEN);
        telemetry.addData("Purple", normPURPLE);

        //add if statements for specific colors added
        /*
        Green, Purple
        GREEN =
        PURPLE =
         */

        return DetectedColor.UNKNOWN;

    }
}