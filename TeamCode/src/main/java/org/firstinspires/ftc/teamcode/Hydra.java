package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Hydra {

    public final float TicksPerInch = 5000/15.5f;

    public Limelight3A limelight;
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor[] motors;
    public DcMotor slide;
    public DcMotor slideTurner;
    public DcMotor lA;
    public DcMotor lA2;
    public Servo toppos,middlepos,bottompos;
    private IMU imu;
    private double targetHeading = 0;
    double lastHeading = 0;
    double changeInHeading;
    float targetDistance = 0;
    long startTicks = 0;
    boolean fwdMoving = false;
    boolean turning = false;
    boolean strafeMoving = false;
    float strafeTargetDistance = 0;


    final double[] forwardDirection = {
            1, 1,
            1, 1};
    final double[] turnDirection = {
            -1, 1,
            -1, 1};
    final double[] strafeDirection = {
            -1, 1,
            1, -1};

    private Telemetry telemetry;
    public ArmController arm;

    public Hydra(Telemetry telemetry){
        arm = new ArmController(this);
        this.telemetry = telemetry;
    }

    public void initializeHardware(HardwareMap hardwareMap){

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        slide = hardwareMap.get(DcMotor.class, "slide");
        slideTurner = hardwareMap.get(DcMotor.class, "slideTurner");
        toppos = hardwareMap.get(Servo.class,"toppos");
        middlepos = hardwareMap.get(Servo.class,"middlepos");
        bottompos = hardwareMap.get(Servo.class,"bottompos");
        lA = hardwareMap.get(DcMotor.class,"lA");
        lA2 = hardwareMap.get(DcMotor.class,"lA2");
        closeClaw();
        middlepos.setPosition(0.5);
        bottompos.setPosition(0.5);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
       // slideTurner.setDirection(DcMotorSimple.Direction.REVERSE);
        slideTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTurner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new DcMotor[]{fl,fr,bl,br};

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

    }

    public void update(){
        double hdgError = 0;
        float fwdError = 0;
        float strafeError = 0;
        double heading = getHeading();
        hdgError = circleDiff(targetHeading, heading);
        double[] powers = {0, 0, 0, 0};

        if(fwdMoving) {
            long pos2 = fl.getCurrentPosition();
            float inches = distance(startTicks, pos2);
            fwdError = targetDistance - inches;
            fwdError = fwdError / Math.abs(targetDistance);
        } else if(strafeMoving){
            long pos2 = bl.getCurrentPosition();
            float inches = distance(startTicks, pos2);
            strafeError = strafeTargetDistance - inches;
            strafeError = strafeError / Math.abs(strafeTargetDistance);
        }

        if(Math.abs(fwdError) < 0.09){
            fwdMoving = false;
        }

        if(Math.abs(strafeError) < 0.09){
            strafeMoving = false;
        }

        if(Math.abs(hdgError) < 0.09){
            turning = false;
        }

        for (int i = 0; i < 4; ++i) {
            powers[i] = clamp(forwardDirection[i] * fwdError, -0.5, 0.5);
            powers[i] += turnDirection[i] * 0;
            powers[i] += clamp(strafeDirection[i] * -strafeError, -0.4, 0.4);
            powers[i] += clamp(turnDirection[i] * -hdgError, -0.5, 0.5);
        }
        for (int i = 0; i < 4; ++i) {
            motors[i].setPower(powers[i]);
        }
        arm.update();
        telemetry.addData("error", fwdError);
        telemetry.addData("position", bl.getCurrentPosition());
        telemetry.addData("strafe error", strafeError);

    }

    private double circleDiff(double a1, double a2) {
        if (a2 > a1 && a2 > 0 && a1 < 0 && Math.abs(a2 - a1) > Math.PI)
            return -((Math.PI - a2) + (a1 + Math.PI));
        else if (a1 > a2 && a1 > 0 && a2 < 0 && Math.abs(a2 - a1) > Math.PI)
            return (Math.PI - a1) + (a2 + Math.PI);
        else
            return a2 - a1;
    }

    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
       return orientation.getYaw(AngleUnit.RADIANS);
    }

    //def clamp(value, min_value, max_value):
     //       if value >  max_value: return max_value
   // elif value < min_value: return min_value
   // return value

    public float clamp (float value, float min_value, float max_value){
        if ( value > max_value) return max_value;
        else if (value < min_value)return min_value;
        return value;
    }

    public double clamp (double value, double min_value, double max_value){
        if ( value > max_value) return max_value;
        else if (value < min_value)return min_value;
        return value;
    }

    public void enableBraking(){
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public float distance(long pos, long pos2){
        long ticks = pos2 - pos;
        return ticks/TicksPerInch;
    }

    public void forward(float p){
        for(int i = 0; i < 4; i++){
            motors[i].setPower(p);
        }
    }

    public void turn(float p){
        final float m[] = {1,-1,1,-1};
        for(int i = 0; i < 4; i++){
            motors[i].setPower(p*m[i]);
        }
    }

    public void turn(double p){
        turn((float)p);
    }

    public void turnTo(double hdg){
        turning = true;
        targetHeading = hdg;
    }

    public void forwardBy(float inches){
        fwdMoving = true;
        targetDistance = inches;
        startTicks = fl.getCurrentPosition();
    }

    public void strafeBy(float inches){
        strafeMoving = true;
        strafeTargetDistance = inches;
        startTicks = bl.getCurrentPosition();
    }

    public boolean isMoving(){
        return fwdMoving || turning || strafeMoving;
    }

    public void stop(){
        forward(0);
    }

    public void openClaw(){
        toppos.setPosition(0);
    }

    public void closeClaw(){
        toppos.setPosition(.45);
    }

    public void autoHome(){
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Extend slide by 200 ticks
        {
            // Start moving the slide the direction you want, wait a moment, then compare the change in distance to your target.
            long startingPosition = slide.getCurrentPosition();

            // Move slide out
            slide.setPower(.2);
            while (true) {
                try{Thread.sleep(50);} catch (Exception e){}
                long currentPosition = slide.getCurrentPosition();
                if (slide.getCurrentPosition() == currentPosition) {
                    // Abort if the slide isn't moving (it hit a mechanical limit somehow)
                    break;
                } else if (Math.abs(currentPosition - startingPosition) >= 200) {
                    // Goal achieved
                    slide.setPower(0.0);
                    break;
                }
            }
        }
        long startPos = slide.getCurrentPosition();
        slide.setPower(-.5);
        while(true){
            try{Thread.sleep(50);} catch (Exception e){}
            if(startPos == slide.getCurrentPosition()){
                break;
            } else{
                startPos = slide.getCurrentPosition();
            }
        }
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slide.setTargetPosition(0);
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.0);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initalis pivet
        startPos = slideTurner.getCurrentPosition();
        slideTurner.setPower(-.5);
        while(true){
            try{Thread.sleep(50);} catch (Exception e){}
            if(startPos == slideTurner.getCurrentPosition()){
                break;
            } else{
                startPos = slideTurner.getCurrentPosition();
            }
        }
        slideTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTurner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideTurner.setPower(0.0);
        slideTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void moveToAprilTag(Telemetry telemetry, Hydra hydra){
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            //telemetry.addData("LL Latency", captureLatency + targetingLatency);
            //telemetry.addData("Parse Latency", parseLatency);
            //telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {

                double yaw = botpose.getOrientation().getYaw();
                double error = result.getTx()*0.01;
                if(error < -1){
                    error = -1;
                }
                if(error > 1){
                    error = 1;
                }
                hydra.turn(error);

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();

        limelight.stop();
    }

}


