package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Hydra {

    public final float TicksPerInch = 5000/15.5f;

    private Limelight3A limelight;
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor[] motors;
    public DcMotor slide;
    public DcMotor slideTurner;
    public Servo toppos,middlepos,bottompos;



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
        toppos.setPosition(0.0);
        middlepos.setPosition(0.5);
        bottompos.setPosition(0.5);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);


        motors = new DcMotor[]{fl,fr,bl,br};

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

    public void stop(){
        forward(0);
    }

    public void autoHome(){
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        long startPos = slide.getCurrentPosition();
        slide.setPower(-.2);
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


