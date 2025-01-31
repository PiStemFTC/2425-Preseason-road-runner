package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAuto", group = "Autonomous", preselectTeleOp = "Drive")
public class TestAuto extends LinearOpMode {
  private Hydra hydra;
  private HydraController hydraController;

  @Override
  public void runOpMode() throws InterruptedException {
    hydra = new Hydra(telemetry);
    hydra.initializeHardware(hardwareMap);
    hydra.enableBraking();
    hydra.autoHome();
    hydraController = new HydraController(hydra);
    waitForStart();
    hydraController.forwardBy(1)
            .turnTo((float)Math.PI/2)
            .forwardBy(67)
            .turnTo((float)(3*Math.PI/4))
            .moveArmToHigh()
            .forwardBy(12)
            .delay(1000)
            .openClawTask()
            .delay(250)
            .forwardBy(-12)
            .moveArmToTravel();

    while(opModeIsActive()){
      hydraController.update();
      hydra.update();

      telemetry.addData("heading", hydra.getHeading());
      telemetry.addData("state", hydraController.toString());
      telemetry.update();
    }
  }
}



