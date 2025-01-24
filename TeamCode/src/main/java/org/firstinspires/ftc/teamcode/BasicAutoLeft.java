package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BasicAutoLeft", group = "Autonomous", preselectTeleOp = "Drive")
public class BasicAutoLeft extends LinearOpMode {
  private Hydra hydra;
  final int Delay = 100;
  final int Terminal = 150;
  private long startTime = 0;
  private long endTime = 0;
  private int state = 0;
  private int nextState = 0;
  public void runOpMode(){
    hydra = new Hydra(telemetry);
    hydra.initializeHardware(hardwareMap);
    hydra.enableBraking();
    hydra.autoHome();
    waitForStart();
    long pos = hydra.fl.getCurrentPosition();
    float targetInches = 24;
    //hydra.forward(0.5f);
    while (opModeIsActive()) {
      switch (state){
        case 0: hydra.forwardBy(1); state = 1; nextState = 2; break;
        //case 1 is a state that waits for the previous action to complete
        case 1: if(!hydra.isMoving())state = nextState; break;
        case 2: hydra.turnTo(Math.PI/2); state = 1; nextState = 3; break;
        case 3: hydra.forwardBy(38); state = 1; nextState = 4; break;
        case 4: hydra.turnTo((3*Math.PI/4)); state = 1; nextState = 5; break;
        case 5: hydra.arm.moveToHigh(); state = 6; nextState = 7; break;
        //wait for arm action to complete
        case 6: if(!hydra.arm.isMoving())state = nextState; break;
        case 7: hydra.forwardBy(12); state = 1; nextState = 8; break;
        //stops for a second
        case 8: delay(1000, nextState = 9); break;
        case 9: hydra.openClaw(); delay(500, nextState = 10); break;
        case 10: hydra.arm.moveToTravel(); state = Terminal; break;
        case Delay: doDelay(); break;
        case Terminal: break;
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
  private void delay(long ms, int nextState){
    long now = System.currentTimeMillis();
    startTime = now;
    endTime = now + ms;
    state = Delay;
    this.nextState = nextState;
  }
  private void doDelay(){
    long now = System.currentTimeMillis();
    if(now >= endTime){
      state = nextState;
    }
  }
}



