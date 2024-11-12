package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BasicAutoRed", group = "Autonomous")
public class BasicAutoRedLeft extends LinearOpMode {
  private Hydra hydra;
    public void runOpMode(){
      hydra = new Hydra();
    hydra.initializeHardware(hardwareMap);
    hydra.enableBraking();
    hydra.autoHome();
      waitForStart();
      long pos = hydra.fl.getCurrentPosition();
      float targetInches = 24;
      //hydra.forward(0.5f);
      while (opModeIsActive()) {
        long pos2 = hydra.fl.getCurrentPosition();
        float inches = hydra.distance(pos,pos2);
        float error = targetInches - inches;
        error = error/targetInches;
        error = Math.min(.7f, error);
        error = Math.max(-.7f, error);
        hydra.forward(error);
        telemetry.addData("pos", pos);
        telemetry.addData("pos2", pos2);
        telemetry.addData("distance", inches);
        telemetry.update();
        //if(inches >= 24){ //15.75in
          //hydra.forward(0);
        //}
      }
}
}



