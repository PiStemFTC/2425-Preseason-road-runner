package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ColorTest", group="TeleOp")
public class ColorSensorTest extends OpMode {
        NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        getDetectedColor(telemetry);
    }

    private enum DetectedColor {
            GREEN,
            PURPLE,
            UNKNOWN
        }

//        public void init(HardwareMap hwMap){
//            colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
//        }

        public DetectedColor getDetectedColor(Telemetry telemetry){
            NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values

            float normGreen, normPurple;
            //normGreen = colors.green / colors.alpha;
            //normPurple = colors.red + colors.blue / colors.alpha; //gets the same reading despite amount of light


            normGreen = colors.green / colors.alpha;
            normPurple = colors.red /colors.alpha; //gets the same reading despite amount of light
            // Example of reading and displaying RGBA values
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Alpha", colors.alpha);


            telemetry.addData("green", normGreen);
            telemetry.addData("purple", normPurple);
            telemetry.update();
            //TODO add if statements for specific colors added

            return DetectedColor.UNKNOWN;
        }
    }

