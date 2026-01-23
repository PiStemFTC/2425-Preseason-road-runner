package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "DONT.CLICK.ME.YET", group = "Autonomous", preselectTeleOp = "Propel")
public class PracticeFarAuto extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private Bessie bessie;
    private BessieController bessieController;
    private ColorSensor colorSensor;
    private int counter;
    private double xError;
    private double yError;
    private int tagID;
    private boolean park = false;

    public enum State {
        Init,
        FindTag,
        Position,
        Launch,
        Intake,
        WaitWhileMoving,
        Done;
    }

    private State state = State.Init;
    private State nextState = State.Done;

    private enum Team {
        Red, Blue
    }

    private Team team = Team.Blue;


    @Override
    public void init() {
        bessie = new Bessie(telemetry);
        bessie.initializeHardware(hardwareMap);
        bessieController = new BessieController(bessie);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight.pipelineSwitch(0); // obelisk aprilTags
        //move start here if limelight has delay
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        bessie.MGRNextLaunchPosition();
        if (gamepad1.x) {
            team = Team.Blue;
        } else {
            team = Team.Red;
        }
        telemetry.addLine(team.toString());
        telemetry.addLine("init complete");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        int tag = -1;
        int targetTx = -7;
        int targetTy = 16;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                tag = fr.getFiducialId();
            }
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx:", llResult.getTx());
            telemetry.addData("Ty:", llResult.getTy());
            //telemetry.addData("Target Area:", llResult.getTa());
            //telemetry.addData("Botpose:", botPose.toString());

            if (state == State.Position && !bessie.isMoving()) {
                xError = llResult.getTx() - targetTx;
                yError = llResult.getTy() - targetTy;
                if (Math.abs(xError * .05) < 1.0 && Math.abs(yError * .05) < 1.0) {
                    state = State.Launch;
                } else {
                    bessieController.lowPower()
                            .forwardBy((float) -yError * .05f)
                            .strafeBy((float) -xError * .05f)
                            .waitWhileMoving();
                    counter++;
                }
            }
        }
        switch (state) {
            case WaitWhileMoving:
                if (bessieController.isDone()) {
                    state = nextState;
                    nextState = State.Done;
                }
                break;
            case Init:
                if (team == Team.Blue) {
                    bessieController
                            .delay(1000)
                            .lift();
                } else {
                    bessieController
                            .delay(1000)
                            .lift();
                }
                state = State.FindTag;
                break;
            case FindTag:
                if (!bessie.isMoving() && tag != -1) {
                    tagID = tag;
                    if (team == Team.Blue) {
                        bessieController
                                .forwardBy(5)
                                .startShooter(.8f)
                                .turnTo((float) (Math.PI / (8.5)));
                    } else {
                        bessieController
                                .forwardBy(5)
                                .startShooter(.8f)
                                .turnTo((float) -(Math.PI / 8.5));
                    }
                    state = State.Position;
                }
                break;
            case Position:
                break;
            case Launch:
                //bessie.shooter.setPower(.2);
                if (true) {
                    bessieController
                            .lift()
                            .startShooter(.825f)
                            .delay(50)
                            // launch artifact 2
                            .mgrNextLaunchPos()
                            .delay(900)
                            .lift()
                            .startShooter(.85f)
                            .delay(50)
                            // launch artifact 3
                            .mgrNextLaunchPos()
                            .delay(900)
                            .lift()
                            .delay(1000)
                            .stopShooter();
                }
                if (park) {
                    if (team == Team.Blue) {
                        bessieController.strafeBy(-20);
                        state = State.Done;
                    } else {
                        bessieController.strafeBy(20);
                        state = State.Done;
                    }
                } else {
                    state = State.WaitWhileMoving;
                    nextState = State.Intake;
                }
                break;
            case Intake:
                if (team == Team.Blue) {
                    bessieController
                            //get lined up with white line
                            .mgrNextIntakePos()
                            .startSpinny(1)
                            .turnTo((float) Math.toRadians(90))
                            .strafeBy(-20)
                            .forwardBy(12)
                            .waitWhileMoving()
                            .delay(350)
                            .mgrNextIntakePos()
                            .delay(350);

                    for (int i = 0; i < 3; i++) {
                        //intake all 3 balls
                        bessieController
                                .forwardBy(3)
                                .waitWhileMoving()
                                .delay(350)
                                .mgrNextIntakePos()
                                .delay(350);
                    }
                    //bessieController.stopSpinny();

                    // move to launch
                    bessieController
                            .turnTo((float) Math.toRadians(45))
                            .startShooter(.6)
                            .mgrNextLaunchPos()
                            .forwardBy(-12)
                            .waitWhileMoving()
                    ;

                } else {
                    bessieController
                            //red side
                            //get lined up with white line
                            .mgrNextIntakePos()
                            .startSpinny(1)
                            .turnTo((float) Math.toRadians(-90))
                            .strafeBy(-20)
                            .forwardBy(12)
                            .waitWhileMoving()
                            .delay(350)
                            .mgrNextIntakePos()
                            .delay(350);

                    for (int i = 0; i < 3; i++) {
                        //intake all 3 balls
                        bessieController
                                .forwardBy(3)
                                .waitWhileMoving()
                                .delay(350)
                                .mgrNextIntakePos()
                                .delay(350);
                    }
                    //bessieController.stopSpinny();

                    // move to launch
                    bessieController
                            .turnTo((float) Math.toRadians(-18))
                            .strafeBy(-10)
                            .delay(1000)
                            .startShooter(.82)
                            .mgrNextLaunchPos()
                            .forwardBy(-12)
                            .waitWhileMoving()
                    ;

                }
                //park = true;
                state = State.WaitWhileMoving;
                nextState = State.Launch;
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Queue Depth", bessieController.queueSize());
        telemetry.addData("X Error", xError);
        telemetry.addData("Y Error", yError);
        bessieController.update();
        bessie.update();
        telemetry.update();
    }
}
