package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SuperCoolAuto", group = "Autonomous", preselectTeleOp = "Drive")
public class SuperCoolAuto extends LinearOpMode {
    private Hydra hydra;
    private HydraController hydraController;

    @Override
    public void runOpMode() throws InterruptedException {
        hydra = new Hydra(telemetry);
        hydra.initializeHardware(hardwareMap);
        hydra.enableBraking();
        hydra.autoHome();
        hydraController = new HydraController(hydra);
        if(gamepad1.dpad_right){ //RIGHT
            hydraController
                    .forwardBy(20)
                    .forwardByAsync(4)
                    .turnToAsync((float) Math.PI / 2)
                    .waitWhileMoving()
                    .forwardBy(46)
                    .turnTo((float) (3 * Math.PI / 4))
                    .moveArmToHigh()
                    .forwardBy(32)
                    .delay(250)
                    .openClawTask()
                    .delay(250)
                    //start get second duck
                    .forwardBy(-6);
            telemetry.addLine("right");
        } else{ //LEFT
            hydraController
                    .forwardByAsync(6)
                    .turnToAsync((float) Math.PI / 2)
                    .waitWhileMoving()
                    .forwardBy(34)
                    .turnTo((float) (3 * Math.PI / 4))
                    .forwardByAsync(4)
                    .moveArmToHigh()
                    .delay(250)
                    .openClawTask()
                    .delay(250)
                    //start get second duck
                    .forwardBy(-9);
            telemetry.addLine("left");
        }
        telemetry.update();
        waitForStart();
        //extending task list now that position is the same despite starting position
        hydraController
                .moveArmToTravel()
                .turnTo(0)
                //.strafeBy(-2)
                .forwardBy(3)
                .moveArmToPick()
                .rotateArmBy(-50)
                .delay(250)
                .closeClawTask()
                .delay(250)
                .moveArmToTravel()
                //drop 2nd duck into basket
                .turnTo((float) (3 * Math.PI / 4))
                .strafeBy(-4)
                .forwardByAsync(7)
                .moveArmToHigh()
                .openClawTask()
                .delay(250)
                .moveArmToTravel()
                //go get 3rd duck
                .turnTo(0)
                .moveArmToPick()
                .strafeBy(-15)
                .forwardBy(7)
                .rotateArmBy(-50)
                .delay(250)
                .closeClawTask()
                .delay(250)
                .moveArmToTravel()
                .turnTo((float) (3 * Math.PI / 4))
                .strafeBy(-6)
                .moveArmToHigh()
                .openClawTask();





        while (opModeIsActive()) {
            hydraController.update();
            hydra.update();

            telemetry.addData("heading", hydra.getHeading());
            telemetry.addData("state", hydraController.toString());
            telemetry.update();
        }
    }
}


