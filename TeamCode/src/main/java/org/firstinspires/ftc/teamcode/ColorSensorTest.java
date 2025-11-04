package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorTest{
        NormalizedColorSensor colorSensor;

        private enum DetectedColor {
            GREEN,
            PURPLE,
            UNKNOWN
        }

        public void init(HardwareMap hwMap){
            colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        }

        public DetectedColor getDetectedColor(Telemetry telemetry){
            NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values

            float normGreen, normPurple;
            normGreen = colors.green / colors.alpha;
            normPurple = colors.red + colors.blue / colors.alpha; //gets the same reading despite amount of light

            telemetry.addData("green", normGreen);
            telemetry.addData("purple", normPurple);

            //TODO add if statements for specific colors added

            return DetectedColor.UNKNOWN;
        }
    }

