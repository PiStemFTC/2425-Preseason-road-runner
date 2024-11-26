package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Drive", group="TeleOp")
public class Drive extends LinearOpMode {

    private IMU imu;
    //public DcMotor slide;


    final double[] forwardDirection = {
            1, 1,
            1, 1};
    final double[] turnDirection = {
            -1, 1,
            -1, 1};
    final double[] strafeDirection = {
            -1, 1,
            1, -1};
    double dT;
    long lastTime;
    long now;
    double targetHeading = 0;
    boolean turning = false;
    double heading;
    double lastHeading;
    double changeInHeading;
    float pivotTgt = 0;
    public DcMotor lA;

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

        int slidePos = 0;
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO XXX MecanumDrive is not available in this version of the repository
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // TODO XXX Create an instance of Hydra (Hydra.java)
        Hydra hydra = new Hydra(telemetry);
        hydra.initializeHardware(hardwareMap);

        // TODO XXX Use the motors mapped by Hydra
        hydra.limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        hydra.limelight.start();

        // TODO XXX Initialize the IMU;
        // * Create an IMU (look up how to accomplish this)
        //   - part of the initialization will be assigning the orientation of the Control Hub
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addLine("zeroing slide");
        telemetry.update();
        hydra.autoHome();
        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        imu.resetYaw();


        while (opModeIsActive()) {
            monkeyBuisness(hydra);
            now = System.currentTimeMillis();
            //dT=Math.subtractExact(lastTime,now);
            dT = now - lastTime;
            lastTime = now;

            hydra.toppos.setPosition(gamepad2.a ? 1.0 : 0.0);
            hydra.middlepos.setPosition((gamepad2.right_stick_x / 2.0) + 0.5);
            hydra.bottompos.setPosition(gamepad2.b ? 1.0 : 0.0);


            // TODO XXX Use the IMU for the heading
            // heading= (drive.pose.heading.toDouble());
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            heading = orientation.getYaw(AngleUnit.RADIANS);
            changeInHeading = circleDiff(lastHeading, heading);
            lastHeading = heading;

            double error = targetHeading - heading;
            double altError = 0;
            if (heading < targetHeading) {
                altError = -((Math.PI - targetHeading) + (Math.PI + heading));
            } else {
                altError = (Math.PI - heading) + (Math.PI + targetHeading);
            }

            if (Math.abs(altError) < Math.abs(error)) {
                error = altError;
            }
            double[] powers = {0, 0, 0, 0};
            for (int i = 0; i < 4; ++i) {
                powers[i] = forwardDirection[i] * -gamepad1.left_stick_y;
                powers[i] += turnDirection[i] * -gamepad1.right_stick_x;
                powers[i] += strafeDirection[i] * -gamepad1.left_stick_x;
                powers[i] += turnDirection[i] * error;
            }
            for (int i = 0; i < 4; ++i) {
                // XXX TODO Use Hydra motors array instead of this copy
                hydra.motors[i].setPower(powers[i]);
            }
            //joepos = (gamepad1.left_stick_x /2.0) +0.5;
            //johnpos = (gamepad1.right_stick_x /2.0) +0.5;
            //joe.setPosition(joepos);
            //john.setPosition(johnpos);
            //telemetry.addData("joe", "%.1f %.1f", gamepad1.left_stick_x,joepos);
            //drive.updatePoseEstimate();
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

            // slidePos = slidePos + (int)(-gamepad2.left_stick_y * 10);

            double power = -gamepad2.left_stick_y;
            slidePos = hydra.slide.getCurrentPosition();
            if (slidePos < 0 && power < 0) {
                power = 0;
            } else if (slidePos > 3000 && power > 0) {
                power = 0;
            } else if (slidePos < 750 && power < 0) {
                power = Math.min(-0.1, power * slidePos / 750.0);

            }
            hydra.slide.setPower(power);

            pivotTgt += gamepad2.right_stick_y * 25;
            pivotTgt = hydra.clamp(pivotTgt,0,1500);
            float pivotError = pivotTgt - hydra.slideTurner.getCurrentPosition();
            pivotError = hydra.clamp(pivotError/100.0f,-1.0f,1.0f);

            hydra.slideTurner.setPower(pivotError);
              //  double power1 = gamepad2.left_stick_x;
            lA = hardwareMap.get(DcMotor.class, "lA");
            power = gamepad2.left_stick_y;
            lA.setPower(power);
            //hydra.slideTurner.setPower(0.2);
                // hydra.slide.setTargetPosition(slidePos);

                orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("heading", (orientation.getYaw(AngleUnit.RADIANS)));
                telemetry.addData("error", error);
                telemetry.addData("altError", altError);
                telemetry.addData("targetHeading", targetHeading);
                telemetry.addData("dT", dT);
                telemetry.addData("changeInHeading", changeInHeading);
                telemetry.addData("pivot tgt", pivotTgt);
            telemetry.addData("pivotError", pivotError);
                if (dT > 0) {
                    telemetry.addData("radians/ms", changeInHeading / dT);
                }
                telemetry.update();
        }
    }
        private void monkeyBuisness(Hydra hydra) {
            LLResult result = hydra.limelight.getLatestResult();

            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());
            }
        }
}