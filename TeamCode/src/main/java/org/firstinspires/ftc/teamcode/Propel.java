package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Propel", group="TeleOp")
public class Propel extends LinearOpMode {

    private IMU imu;
    public DistanceSensor distanceSensor;

    //public DcMotor slide;


    final double[] forwardDirection = {
            1, 1,
            1, 1};
    final double[] turnDirection = {
            -1, 1,
            -1, 1};
    final double[] strafeDirection = {
            -1, 1,
            1,-1};
    double dT;
    long lastTime;
    long now;
    double targetHeading = 0;
    boolean turning = false;
    double heading;
    double lastHeading;
    double changeInHeading;
    boolean dpadUpAlreadyPressed = false;
    boolean dpadDownAlreadyPressed = false;
    double distance;
    int velocity = -1;

    private double circleDiff(double a1, double a2) {
        if (a2 > a1 && a2 > 0 && a1 < 0 && Math.abs(a2 - a1) > Math.PI)
            return -((Math.PI - a2) + (a1 + Math.PI));
        else if (a1 > a2 && a1 > 0 && a2 < 0 && Math.abs(a2 - a1) > Math.PI)
            return (Math.PI - a1) + (a2 + Math.PI);
        else
            return a2 - a1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        long ticks = 0;
        boolean launchEnabled = false;

        int slidePos = 0;
        if (velocity == -1){
          velocity = 1000;
        }
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bessie bessie = new Bessie(telemetry);
        bessie.initializeHardware(hardwareMap);

        bessie.limelight.pipelineSwitch(0);
        bessie.limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        telemetry.update();

        bessie.MGRNextIntakePosition();

        waitForStart();
        imu.resetYaw();

        boolean grab = true;
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            bessie.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            if (gamepad2.left_bumper) {
                bessie.spinny.setPower(-1);
            } else if (gamepad2.left_trigger > .1  && bessie.mgrMode == Bessie.MGRMode.INTAKE) {
                bessie.spinny.setPower(1);
            } else {
                bessie.spinny.setPower(0);
            }

            //if (gamepad2.right_bumper) {
            //    bessie.shooter.setPower(.65);
            //} else {
            //    bessie.shooter.setPower(0);
            //}

            double encoderTicksPerRev = 28;
            double ticksPerSecond = bessie.shooter.getVelocity();
            double shooterRPM = (ticksPerSecond / encoderTicksPerRev) * 60.0;
            telemetry.addData("Shooter RPM", shooterRPM);

           // if (gamepad2.a) {
             //   bessie.MGR.setPower(.15);
               // bessie.MGR.setDirection(com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE);
            //} else if (gamepad2.y) {
              //  bessie.MGR.setPower(-.15);
            //} else {
              //  bessie.MGR.setPower(0);
            //}

            //if(gamepad2.dpad_up && !dpadUpAlreadyPressed){
            if (gamepad2.dpadUpWasPressed()) {
                launchEnabled = true;
                bessie.MGRNextLaunchPosition();
            }// else if(!gamepad2.dpad_up){ dpadUpAlreadyPressed = false; }

            //if(gamepad2.dpad_down && !dpadDownAlreadyPressed){
            if (gamepad2.dpadDownWasPressed()) {
                launchEnabled = false;
                bessie.MGRNextIntakePosition();
            }// else if(!gamepad2.dpad_down){ dpadDownAlreadyPressed = false; }

            if(gamepad2.dpadLeftWasPressed()){
                velocity -= 100;
            }

            if(gamepad2.dpadRightWasPressed()){
                velocity += 100;
            }

            if (launchEnabled) {
                bessie.shooter.setVelocity(velocity);
            } else {
                bessie.shooter.setVelocity(0);
            }

            if(gamepad2.x){
                bessie.flicky.setPosition(.5);
            } else{
                bessie.flicky.setPosition(0.02);
            }

            //if(gamepad1.y){
                //bessie.startReadColors();
            //}

            telemetry.addData(String.valueOf(bessie.analogInput.getMaxVoltage()), "max voltage");
            telemetry.addData(String.valueOf(bessie.analogInput.getVoltage()), "voltage");
            telemetry.addData(String.valueOf(velocity), "Velocity");

            now = System.currentTimeMillis();
            //dT=Math.subtractExact(lastTime,now);
            dT = now - lastTime;
            lastTime = now;

            // TODO XXX Use the IMU for the heading
            // heading= (drive.pose.heading.toDouble());
            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            heading = orientation.getYaw(AngleUnit.RADIANS);
            changeInHeading = circleDiff(lastHeading, heading);
            lastHeading = heading;

            double error = circleDiff(heading,targetHeading);
            //double error = targetHeading - heading;
            /*
            double altError = 0;
            if (heading < targetHeading) {
                altError = -((Math.PI - targetHeading) + (Math.PI + heading));
            } else {
                altError = (Math.PI - heading) + (Math.PI + targetHeading);
            }

            if (Math.abs(altError) < Math.abs(error)) {
                error = altError;
            }
             */
            double[] powers = {0, 0, 0, 0};
            for (int i = 0; i < 4; ++i) {
                powers[i] = forwardDirection[i] * -(gamepad1.left_stick_y * .6);
                powers[i] += turnDirection[i] * -(gamepad1.right_stick_x * .6);
                powers[i] += strafeDirection[i] * -(gamepad1.left_stick_x * .6);
                powers[i] += turnDirection[i] * error;
            }

            for (int i = 0; i < 4; ++i) {
                bessie.motors[i].setPower(powers[i]);
            }

            if (gamepad1.right_stick_x != 0.0) {
                //targetHeading=(drive.pose.heading.toDouble());
                orientation = imu.getRobotYawPitchRollAngles();
                targetHeading = orientation.getYaw(AngleUnit.RADIANS);
                turning = true;
            } else {
                //not turning.
                if (turning) {
                    //sleep(200);
                    //drive.updatePoseEstimate();
                    orientation = imu.getRobotYawPitchRollAngles();
                    targetHeading = (orientation.getYaw(AngleUnit.RADIANS) + ((changeInHeading / dT) * 100));
                }
                turning = false;
            }


            if (ticks % 20 == 0) {
                bessie.colorSensor();
                bessie.ballColorIndicators();
            }
            ticks++;
            bessie.updateMGR();
            orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Color", bessie.currentColor);
            telemetry.addData("heading", (orientation.getYaw(AngleUnit.RADIANS)));
            telemetry.addData("error", error);
            //telemetry.addData("altError", altError);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("dT", dT);
            telemetry.addData("changeInHeading", changeInHeading);
            if (dT > 0) {
                telemetry.addData("radians/ms", changeInHeading / dT);
            }
            monkeyBuisness(bessie);
            telemetry.update();
        }
    }

    public double getDistanceFromTag(double ta){
    double scale = 5002.399;
    //double distance = (scale*Math.pow(ta,-2));

    double distance = 14.62196 + (450.9122 - 14.62196)/(1 + Math.pow(ta/0.06846392, 0.7124414));
    return distance;
    }

    public double getVelocityFromDistance(double distance) {
        return 8.249527 * distance + 1068.729;
    }

    private void monkeyBuisness(Bessie bessie) {
        LLResult result = bessie.limelight.getLatestResult();

        if (result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Target Area:", result.getTa());
            distance = getDistanceFromTag(result.getTa());
            velocity = (int)getVelocityFromDistance(distance);
            telemetry.addData("Distance", distance);
            telemetry.addData("Botpose", botpose.toString());

        }
    }
}