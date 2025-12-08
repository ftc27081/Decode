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
    public void init (HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor"); //Put name of sensor in control hub in quotations
        colorSensor.setGain(5);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //will return the 4 earlier values

        float normGreen, normPurple;
        normGreen = colors.green / colors.alpha;
        normPurple = colors.red + colors.blue / colors.alpha;

        telemetry.addData("green", normGreen);
        telemetry.addData("purple", normPurple);

        //add if statements for specific colors added
        /*
        Green, Purple
        GREEN = > .4, <.3,                                      // ADD the restriction shownd with the adjusted set gain.
        PURPLE = > .4, <.3,

        if(normPurple > 0.4 && normGreen <0.3){
            return DetectedColor.PURPLE;                       // change numers here aswell
        } else if (normPurple < 0.3 && normGreen > 0.4){
        return DetectedColor.GREEN;                             //Once imported all the numbers and adjusted the if commanments. Uncomment these lines and comment out lines 28-29
        }
        else if */{         return DetectedColor.UNKNOWN;
        }

    }
}