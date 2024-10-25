package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
            hydra.toppos.setPosition(gamepad1.a? 1.0 : 0.0 );
            hydra.middlepos.setPosition((gamepad1.right_stick_x /2.0) +0.5);
            hydra.bottompos.setPosition((gamepad1.left_stick_x /2.0) +0.5);
          // hydra.moveToAprilTag(telemetry, hydra);

        }
    }
}

