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

@Autonomous(name="TheCoolestAuto", group="Autonomous")
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
    public enum State {
        Init,
        FindTag,
        Position,
        Done;
    }
    private State state = State.Init;


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
        if(llResult != null && llResult.isValid()){
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
                if(Math.abs(xError * .05) < 1.0 && Math.abs(yError * .05) < 1.0){
                    state = State.Done;
                } else {
                    bessieController.lowPower()
                            .forwardBy((float) -yError * .05f)
                            .strafeBy((float) -xError * .05f)
                            .waitWhileMoving();
                    counter++;
                }
            }
        }
        switch (state){
            case Init:
                bessieController
                        .forwardBy(30)
                        .turnTo((float) (Math.PI / 2));
                state = State.FindTag;
                break;
            case FindTag:
                if(tag != -1) {
                    tagID = tag;
                    bessieController
                            .turnTo((float) Math.PI);
                    state = State.Position;
                }
                break;
            case Position:
                break;
        }
        telemetry.addData("State", state);
        telemetry.addData("Counter", counter);
        telemetry.addData("X Error", xError);
        telemetry.addData("Y Error", yError);
        bessieController.update();
        bessie.update();
        telemetry.update();
    }
}
