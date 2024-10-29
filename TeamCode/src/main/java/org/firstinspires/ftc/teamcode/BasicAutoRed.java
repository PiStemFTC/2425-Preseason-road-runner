package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BasicAutoRed", group = "Autonomous")
public class BasicAutoRed extends LinearOpMode {
  private Hydra hydra;
    public void runOpMode(){
    hydra.initializeHardware(hardwareMap);
    hydra.forward(5);
}
}



