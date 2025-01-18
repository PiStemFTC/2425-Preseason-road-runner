package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BasicAutoRight", group = "Autonomous")
public class BasicAutoRight extends LinearOpMode {
    private Hydra hydra;
    public void runOpMode(){
        hydra = new Hydra(telemetry);
        hydra.initializeHardware(hardwareMap);
        hydra.enableBraking();
        hydra.autoHome();
        waitForStart();
        long pos = hydra.fl.getCurrentPosition();
        float targetInches = 24;
        //hydra.forward(0.5f);
        int state = 0;
        int nextState = 0;
        while (opModeIsActive()) {
            switch (state){
                case 0: hydra.forwardBy(1); state = 1; nextState = 2; break;
                //case 1 is a state that waits for the previous action to complete
                case 1: if(!hydra.isMoving())state = nextState; break;
                case 2: hydra.turnTo(Math.PI/2); state = 1; nextState = 3; break;
                case 3: hydra.forwardBy(65); state = 1; nextState = 4; break;
                case 4: hydra.turnTo((3*Math.PI/4)); state = 1; nextState = 5; break;
                case 5: hydra.arm.moveToHigh(); state = 6; nextState = 7; break;
                //wait for arm action to complete
                case 6: if(!hydra.arm.isMoving())state = nextState; break;
                case 7: hydra.forwardBy(12); state = 1; nextState = 8; break;
                case 8: hydra.openClaw(); break;
            }
            hydra.update();
            //hydra.forward(error);
            telemetry.addData("heading", hydra.getHeading());
            telemetry.addData("pos", pos);
            telemetry.update();
            //if(inches >= 24){ //15.75in
            //hydra.forward(0);
            //}
        }
    }
}



