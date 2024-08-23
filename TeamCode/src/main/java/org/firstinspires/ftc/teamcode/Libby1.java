package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Libby1", group="Autonomous")
public class Libby1 extends LinearOpMode {
    @Override
    public void runOpMode(){
        MecanumDrive drive =new MecanumDrive(hardwareMap,new Pose2d(11.8,61.7,Math.toRadians(90)));
        Action traj1;
        traj1=drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(180))
                .build();
        //while (!isStopRequested() && !opModeIsActive()) {

       // }
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(new SequentialAction(traj1));
}
}

