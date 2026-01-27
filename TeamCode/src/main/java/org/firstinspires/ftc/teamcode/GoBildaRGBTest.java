package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GoBILDA RGB Test")
public class GoBildaRGBTest extends LinearOpMode {

    Servo RGB;

    @Override
    public void runOpMode() {

        // Name must match the Servo name in Robot Configuration
        RGB = hardwareMap.get(Servo.class, "RGB");

        waitForStart();


        RGB.setPosition(0.66);


        while (opModeIsActive()) {
            // LED stays on, nothing else needed
        }
    }
}


