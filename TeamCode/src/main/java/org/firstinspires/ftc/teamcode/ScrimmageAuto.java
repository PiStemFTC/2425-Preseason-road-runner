package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ScrimmageAuto", group = "Autonomous", preselectTeleOp = "Propel")
public class ScrimmageAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bessie bessie;
        BessieController bessieController;
        bessie = new Bessie(telemetry);
        bessie.initializeHardware(hardwareMap);
        bessie.enableBraking();
        bessieController = new BessieController(bessie);
        waitForStart();
        bessieController.forwardBy(1)
                .forwardBy(2);

        while(opModeIsActive()){
            bessieController.update();
            bessie.update();

            telemetry.addData("heading", bessie.getHeading());
            telemetry.addData("state", bessieController.toString());
            telemetry.update();
        }
    }
}



