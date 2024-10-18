package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "HydraTeleOp", group = "TeleOp")
public class HydraTeleOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        Hydra hydra = new Hydra();
        hydra.initializeHardware(hardwareMap);
        waitForStart();
        //hydra.forward(0.1f);
        //hydra.turn(0.1f);
        while(opModeIsActive()){
           hydra.moveToAprilTag(telemetry, hydra);

        }
    }
}

