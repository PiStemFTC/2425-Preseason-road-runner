package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "EvenCoolerAuto", group = "Autonomous", preselectTeleOp = "Drive")
public class EvenCoolerAuto extends LinearOpMode {
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
                    .delay(50)
                    .openClawTask()
                    .delay(50)
                    //start get second duck
                    .forwardBy(-6);
            telemetry.addLine("right");
        } else{ //LEFT
            hydraController
                    .forwardByAsync(8)
                    .turnToAsync((float) Math.PI / 2)
                    .waitWhileMoving()
                    .forwardBy(32)
                    .turnTo((float) (3 * Math.PI / 4))
                    .moveArmToHigh()
                    .forwardBy(4)
                    .delay(50)
                    .openClawTask()
                    .delay(50)
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
                //strafe for grabbing 2nd duck
                .strafeBy(-2.5f)
                //.lowPower()
                //.forwardBy(.5f)
                //.highPower()
                .moveArmToPick()
                .rotateArmBy(-75)
                .delay(50)
                .closeClawTask()
                .delay(50)
                .moveArmToTravel()
                //drop 2nd duck into basket
                .turnTo((float) (3 * Math.PI / 4))
                .strafeBy(-3)
                .moveArmToHigh()
                .forwardBy(7)
                .openClawTask()
                .delay(50)
                .forwardBy(-5)
                .moveArmToTravel()
                //go get 3rd duck
                .turnTo(0)
                .moveArmToPick()
                //strafe to grab 3rd duck
                .strafeBy(-9.5f)
                .forwardBy(2.5f)
                .rotateArmBy(-75)
                .delay(50)
                .closeClawTask()
                .delay(50)
                .moveArmToTravel()
                //drop 3rd duck into basket
                .turnTo((float) (3 * Math.PI / 4))
                .strafeBy(-9)
                .moveArmToHigh()
                .lowPower()
                .forwardBy(2)
                .highPower()
                .delay(50)
                .openClawTask()
                .delay(50)
                .rotateArmBy(100);
        //.moveArmToTravel();
        //.forwardBy(-10);

        while (opModeIsActive()) {
            hydraController.update();
            hydra.update();
            telemetry.addData("heading", hydra.getHeading());
            telemetry.addData("state", hydraController.toString());
            telemetry.update();
        }
    }
}


