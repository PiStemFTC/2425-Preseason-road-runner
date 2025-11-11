package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "ColorDetection", group = "TeleOp")
public class ColorDetection extends OpMode {

    // Declare Color Sensor
    private ColorSensor colorSensor;

    @Override
    public void init() {
        // Initialize the color sensor
        colorSensor = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);  // Turn on the LED light for accurate color detection
    }

    @Override
    public void loop() {
        // Get RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Convert RGB to HSV
        float[] hsv = new float[3];
        RGBtoHSV(red, green, blue, hsv);

        // Extract Hue, Saturation, and Value
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Add telemetry data to monitor RGB, HSV values
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.addData("Hue", hue);
        telemetry.addData("Saturation", saturation);
        telemetry.addData("Value", value);

        // Adjusted color detection logic
        // Detect Green color (hue range 90 - 160, saturation > 0.4, value > 0.2)
        if (hue >= 90 && hue <= 160 && saturation > 0.4 && value > 0.2) {
            telemetry.addData("Detected Color", "Green");
        }
        // Detect Purple color (hue range 230 - 280, saturation > 0.4, value > 0.2)
        else if (hue >= 230 && hue <= 280 && saturation > 0.4 && value > 0.2) {
            telemetry.addData("Detected Color", "Purple");
        }
        // If no color is detected, mark as Unknown
        else {
            telemetry.addData("Detected Color", "Unknown");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Nothing to stop, no motors running
    }

    // Method to convert RGB to HSV
    public void RGBtoHSV(int r, int g, int b, float[] hsv) {
        float max = Math.max(Math.max(r, g), b);
        float min = Math.min(Math.min(r, g), b);
        float delta = max - min;

        float h = 0;
        float s = (max == 0) ? 0 : (delta / max);
        float v = max / 255.0f;

        if (max == min) {
            h = 0;  // No hue
        } else {
            if (max == r) {
                h = (g - b) / delta;  // Red is max
            } else if (max == g) {
                h = (b - r) / delta + 2;  // Green is max
            } else {
                h = (r - g) / delta + 4;  // Blue is max
            }

            h *= 60;  // Convert to degrees
            if (h < 0) h += 360;  // Ensure positive hue
        }

        hsv[0] = h;  // Hue
        hsv[1] = s;  // Saturation
        hsv[2] = v;  // Value
    }
}
