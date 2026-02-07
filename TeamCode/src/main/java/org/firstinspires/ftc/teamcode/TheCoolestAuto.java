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

@Autonomous(name="GoalAuto", group="Autonomous", preselectTeleOp = "Propel")
public class TheCoolestAuto extends OpMode {
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
    private boolean useCamera = false;
    public enum State {
        Init,
        FindTag,
        Position,
        Launch,
        Intake,
        Done;
    }
    private State state = State.Init;
    private enum Team{
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
        if(gamepad1.x){
            team = Team.Blue;
        } else{
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
        if (useCamera) {
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
                        useCamera = false;
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
        }
        switch (state) {
            case Init:
                //bessieController.lift().delay(500).startReadColors();
                if (team == Team.Blue) {
                    bessieController
                            .delay(250)
                            .lift()
                            .forwardBy(30)
                            .turnTo((float) (Math.PI / 2));
                } else {
                    bessieController
                            .delay(250)
                            .lift()
                            .forwardBy(30)
                            .turnTo((float) -(Math.PI / 2));
                }
                state = State.FindTag;
                break;
            case FindTag:
                useCamera = true;
                if (tag != -1) {
                    tagID = tag;
                    if (team == Team.Blue) {
                        bessieController
                                .startShooter(.6f)
                                .turnTo((float) Math.PI - .1f);
                    } else {
                        bessieController
                                .startShooter(.6f)
                                .turnTo((float) -(Math.PI - .1f));
                    }
                    state = State.Position;
                }
                break;
            case Position:
                break;
            case Launch:
//                    bessieController
//                            .delay(2500)
//                            .lift()
//                            .delay(2500)
//                            //.lift()
//                            .delay(2500)
//                            //.log("Done");
//                    ;

                //bessieController.stopReadColors().mgrNextLaunchPos();
                if (true) {
                    bessieController
                            .lift()
                            .delay(200)
                            .startShooter(.625f)
                            .delay(50)
                            // launch artifact 2
                            .mgrNextLaunchPos()
                            .delay(600)
                            .lift()
                            .startShooter(.65f)
                            .delay(50)
                            // launch artifact 3
                            .mgrNextLaunchPos()
                            .delay(600)
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
                    state = State.Intake;
                }
                break;
            case Intake:
                if (team == Team.Blue) {
                    bessieController
                            //line up with white line
                            .mgrNextIntakePos()
                            .startSpinny(1)
                            .turnTo((float) Math.toRadians(-140.0))
                            .strafeBy(-20)
                            .forwardBy(8)
                            .waitWhileMoving()
                            .delay(350)
                            .mgrNextIntakePos()
                            .delay(350);

                    for (int i = 0; i < 2; i++) {
                        bessieController
                                //intake all 3 balls
                                .forwardBy(2)
                                .waitWhileMoving()
                                .delay(350)
                                .mgrNextIntakePos()
                                .delay(350);
                    }
                    //bessieController.stopSpinny();

                    // move to launch blue
                    bessieController
                            .turnTo((float) Math.toRadians(175))
                            .startShooter(.6)
                            .mgrNextLaunchPos()
                            .strafeBy(34)
                            .waitWhileMoving()
                    ;

                } else {
                    bessieController
                            //line up to line red
                            .mgrNextIntakePos()
                            .startSpinny(1)
                            .turnTo((float) Math.toRadians(-220.0))
                            .strafeBy(20)
                            .forwardBy(8)
                            .waitWhileMoving()
                            .delay(350)
                            .mgrNextIntakePos()
                            .delay(350);

                    for (int i = 0; i < 2; i++) {
                        bessieController
                                //intake all 3 balls
                                .forwardBy(3)
                                .waitWhileMoving()
                                .delay(350)
                                .mgrNextIntakePos()
                                .delay(350);
                    }
                    //bessieController.stopSpinny();

                    // move to launch
                    bessieController
                            .turnTo((float) Math.toRadians(-175))
                            .startShooter(.6)
                            .mgrNextLaunchPos()
                            .strafeBy(-34)
                            .waitWhileMoving()
                    ;

                }
                park = true;
                state = State.Launch;
                break;
        }

            telemetry.addData("State", state);
            telemetry.addData("Counter", counter);
            telemetry.addData("X Error", xError);
            telemetry.addData("Y Error", yError);
            //telemetry.addData("Colors", bessie.allColors[0]+","+bessie.allColors[1]+","+bessie.allColors[2]);
            bessieController.update();
            bessie.update();
            telemetry.update();
        }
        }
