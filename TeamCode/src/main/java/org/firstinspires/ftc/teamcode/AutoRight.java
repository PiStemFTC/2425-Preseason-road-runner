package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRight", group = "Autonomous", preselectTeleOp = "Drive")
public class AutoRight extends LinearOpMode {
    private Hydra hydra;
    private HydraController hydraController;

    @Override
    public void runOpMode() throws InterruptedException {
        hydra = new Hydra(telemetry);
        hydra.initializeHardware(hardwareMap);
        hydra.enableBraking();
        hydra.autoHome();
        hydraController = new HydraController(hydra);
        if(gamepad1.dpad_right){
            hydraController.forwardBy(11)
                    .turnTo((float) Math.PI / 2)
                    .forwardBy(50)
                    .turnTo((float) (3 * Math.PI / 4))
                    .moveArmToHigh()
                    .forwardBy(30);
            telemetry.addLine("right");
        } else{
            hydraController.forwardBy(2)
                    .turnTo((float) Math.PI / 2)
                    .forwardBy(28)
                    .turnTo((float) (3 * Math.PI / 4))
                    .moveArmToHigh()
                    .forwardBy(12);
            telemetry.addLine("left");
        }
        telemetry.update();
        waitForStart();
        //extending task list now that position is the same despite starting position
        hydraController.delay(1000)
                .openClawTask()
                .delay(250)
                .forwardBy(-12)
                .moveArmToTravel();

        while (opModeIsActive()) {
            hydraController.update();
            hydra.update();

            telemetry.addData("heading", hydra.getHeading());
            telemetry.addData("state", hydraController.toString());
            telemetry.update();
        }
    }
}


