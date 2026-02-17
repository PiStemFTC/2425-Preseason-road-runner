package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

public class Bessie {

    public final float TicksPerInch = 5000/15.5f;

    public Limelight3A limelight;
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor[] motors;
    public DcMotor spinny;
    public DcMotorEx shooter;
    public Servo flicky;
    public CRServo MGR;
    public Servo RGB;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    private IMU imu;
    public AnalogInput analogInput;
    private double targetHeading = 0;
    double lastHeading = 0;
    double changeInHeading;
    float targetDistance = 0;
    long startTicks = 0;
    boolean fwdMoving = false;
    boolean turning = false;
    boolean strafeMoving = false;
    float strafeTargetDistance = 0;
    public float fwdPower = .6f;
    public double MGRTargetVoltage = 0;
    public int MGRPositionIndex = 0;
    public double tx, ty, ta, bearingToTarget;
    public boolean limelightValid = false;
    public String currentColor = "unknown";
    public PredominantColorProcessor chamberControlColor, chamberBarrelColor, chamberExpansionColor;
    public PredominantColorProcessor.Result resultControl, resultBarrel, resultExpansion;
    public String motifOrder;
    public String[] chambers = new String[3];

    //private org.firstinspires.ftc.teamcode.subsystems.RTPAxon axon;
    public enum MGRMode {
        INTAKE, LAUNCH
    }
    public MGRMode mgrMode;
    public MGRController mgrController;

    public Telemetry telemetry;

    final double[] forwardDirection = {
            1, 1,
            1, 1};
    final double[] turnDirection = {
            -1, 1,
            -1, 1};
    final double[] strafeDirection = {
            -1, 1,
            1, -1};

    public Bessie(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void initializeHardware(HardwareMap hardwareMap){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        MGR = hardwareMap.get(CRServo.class, "MGR");
        flicky = hardwareMap.get(Servo.class, "flicky");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        analogInput = hardwareMap.get(AnalogInput.class, "analogInput");
        colorSensor = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        RGB = hardwareMap.get(Servo.class, "RGB");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        spinny.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mgrController = new MGRController(1.0, .01, .07);

        //axon = new org.firstinspires.ftc.teamcode.subsystems.RTPAxon(MGR, analogInput);
        //axon.setMaxPower(0.1);  // Limit max power to 50%
        //axon.setPidCoeffs(1.0, 0.0005, 0.0025);  // Set PID coefficients

        motors = new DcMotor[]{fl,fr,bl,br};

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

    }

    public void webcam(HardwareMap hardwareMap) {
        chamberControlColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(1, 215, 180, 480)) //left ball
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();
        chamberBarrelColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(500, 200, 640,480))//right ball
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();
        chamberExpansionColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(250, 130, 320, 250))//launch ball
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();

    VisionPortal myVisionPortal;
        myVisionPortal = new VisionPortal.Builder()
                .addProcessor(chamberBarrelColor)
                .addProcessor(chamberControlColor)
                .addProcessor(chamberExpansionColor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //.enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public String[] getChambers()
    {
        //String[] chambers = new String[3];          //0 = Barrel, 1 = Control, 2 = Expansion

        String resultBarrel = chamberBarrelColor.getAnalysis().closestSwatch.toString();
        String resultControl = chamberControlColor.getAnalysis().closestSwatch.toString();
        String resultExpansion = chamberExpansionColor.getAnalysis().closestSwatch.toString();

        String returnBarrel = "?";
        String returnControl = "?";
        String returnExpansion = "?";

        boolean testingSwatchOn = false; // turns on swatch value to return values

        if(resultBarrel.equalsIgnoreCase("GREEN") || resultBarrel.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnBarrel = "GREEN";
        }
        else if(resultBarrel.equalsIgnoreCase("PURPLE") || resultBarrel.equalsIgnoreCase("ARTIFACT_PURPLE")
                || resultBarrel.equalsIgnoreCase("MAGENTA") || resultBarrel.equalsIgnoreCase("BLUE"))
        {
            returnBarrel = "PURPLE";
        }
        else
        {
            returnBarrel = "EMPTY";
        }

        if(resultControl.equalsIgnoreCase("GREEN") || resultControl.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnControl = "GREEN";
        }
        else if(resultControl.equalsIgnoreCase("PURPLE") || resultControl.equalsIgnoreCase("ARTIFACT_PURPLE")
                || resultControl.equalsIgnoreCase("MAGENTA") || resultControl.equalsIgnoreCase("BLUE"))
        {
            returnControl = "PURPLE";
        }
        else
        {
            returnControl = "EMPTY";
        }

        if(resultExpansion.equalsIgnoreCase("GREEN") || resultExpansion.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnExpansion = "GREEN";
        }
        else if(resultExpansion.equalsIgnoreCase("PURPLE") || resultExpansion.equalsIgnoreCase("ARTIFACT_PURPLE")
                || resultExpansion.equalsIgnoreCase("MAGENTA") || resultExpansion.equalsIgnoreCase("BLUE"))
        {
            returnExpansion = "PURPLE";
        }
        else
        {
            returnExpansion = "EMPTY";
        }

        if(testingSwatchOn)
        {
            returnBarrel += " - " + resultBarrel;
            returnControl += " - " + resultControl;
            returnExpansion += " - " + resultExpansion;
        }
        chambers[0]  = returnExpansion;
        chambers[1]  = returnControl;
        chambers[2]  = returnBarrel;

        return chambers;
    }

    public double[] axonToOrder(double axonTargetCurrent)
    {
        double[] axonReturn = new double[2];
        double axonTarget = axonTargetCurrent;
        double axonDirection = 0;//1.0 = cw, -1.0 = ccw
        chambers = getChambers();
        String chamberOrder = chambers[0].substring(0, 1) + chambers[1].substring(0, 1) + chambers[2].substring(0, 1);

        if(chamberOrder.toLowerCase().contains("e"))
        {
            // If one chamber appears empty, assume the empty is the missing color
            if(chamberOrder.equalsIgnoreCase("epp") || chamberOrder.equalsIgnoreCase("gep") || chamberOrder.equalsIgnoreCase("gpe"))
            {
                chamberOrder = "gpp";
            }
            else if(chamberOrder.equalsIgnoreCase("egp") || chamberOrder.equalsIgnoreCase("pep") || chamberOrder.equalsIgnoreCase("pge"))
            {
                chamberOrder = "pgp";
            }
            else if(chamberOrder.equalsIgnoreCase("epg") || chamberOrder.equalsIgnoreCase("peg") || chamberOrder.equalsIgnoreCase("ppe"))
            {
                chamberOrder = "ppg";
            }

        }

        if(chamberOrder.toLowerCase().contains("e"))
        {
            // More than one chamber registered as empty
            axonTarget = ((axonTarget + 360) + 240) % 360;
            axonDirection = -2.0;
        }
        else if(motifOrder.equalsIgnoreCase("gpp"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                //nothing
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
        }
        else if(motifOrder.equalsIgnoreCase("pgp"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                //nothing
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
        }
        else if(motifOrder.equalsIgnoreCase("ppg"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                //nothing
            }
            telemetry.addData(chamberOrder, "Chamber Order:");
        }

        axonReturn[0] = axonTarget;
        axonReturn[1] = axonDirection;

        return axonReturn;
    }

    public void updateMGR() {
        MGR.setPower(mgrController.calculate(
                analogInput.getVoltage()));
        //axon.update();
    }

    public void updateMGR_(){
        double MGRpos = 0;
        double MGRerror = 0;
        //MGR position update
        MGRpos = analogInput.getVoltage();
        MGRerror = MGRDiff(MGRTargetVoltage, MGRpos);

        if(MGRerror < -0.005){
            double p = -MGRerror*1.2;
            if(p > 0.15){
                p = 0.15;
            }
            MGR.setPower(-p);
        }

        if(MGRerror > 0.005){
            double p = -MGRerror*1.2;
            if(p < -0.15){
                p = -0.15;
            }
            MGR.setPower(-p);
        }

        telemetry.addData("index", MGRPositionIndex);
        telemetry.addData("MGR error", MGRerror);
        telemetry.addData("MGR position", MGRpos);
        telemetry.addData("MGR target voltage", MGRTargetVoltage);
    }

    public void update(){
        updateLimelight();
        double hdgError = 0;
        float fwdError = 0;
        float strafeError = 0;
        double heading = getHeading();
        hdgError = circleDiff(targetHeading, heading);
        double[] powers = {0, 0, 0, 0};

        if(fwdMoving) {
            long pos2 = -fr.getCurrentPosition();
            float inches = distance(startTicks, pos2);
            fwdError = targetDistance - inches;
            fwdError = fwdError / Math.abs(targetDistance);
        } else if(strafeMoving){
            long pos2 = -br.getCurrentPosition();
            float inches = distance(startTicks, pos2);
            strafeError = strafeTargetDistance - inches;
            strafeError = strafeError / Math.abs(strafeTargetDistance);
        }

        if(Math.abs(fwdError) < 0.09){
            fwdMoving = false;
        }

        if(Math.abs(strafeError) < 0.2){
            strafeMoving = false;
        }

        if(Math.abs(hdgError) < 0.09){
            turning = false;
        }

        for (int i = 0; i < 4; ++i) {
            powers[i] = clamp(forwardDirection[i] * fwdError, -fwdPower, fwdPower);
            powers[i] += turnDirection[i] * 0;
            powers[i] += clamp(strafeDirection[i] * -strafeError, -0.4, 0.4);
            powers[i] += clamp(turnDirection[i] * -hdgError, -0.5, 0.5);
        }
        for (int i = 0; i < 4; ++i) {
            motors[i].setPower(powers[i]);
        }

        updateMGR();
        telemetry.addData("error", fwdError);
        telemetry.addData("position", -fr.getCurrentPosition());
        telemetry.addData("strafe error", strafeError);

    }

    public void ballColorIndicators(boolean tagVisible){
        if(mgrMode == MGRMode.INTAKE) {
            if ((distanceSensor.getDistance(DistanceUnit.CM) < 10)) {
                RGB.setPosition(0.3); //blue
            } else {
                RGB.setPosition(0.93); //white
            }
        }else{
            //launch
            if(tagVisible) {
                if (currentColor.equals("green")) {
                    RGB.setPosition(0.5); //green
                } else if (currentColor.equals("purple")) {
                    RGB.setPosition(0.64); //purple
                } else {
                    RGB.setPosition(0.93); //white
                }
            }else{
                RGB.setPosition(0.0);
            }
        }

        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
    }

    private double MGRCalcIntakePosition(int position){
        double positions[] = {0.316, 1.413, 2.506};
        //double positions[] = {0.0+60.0, 120.0+60.0, 240.0+60.0};
        return positions[position];
    }

    public void MGRNextIntakePosition(){
        mgrMode = MGRMode.INTAKE;
        MGRPositionIndex++;
        if(MGRPositionIndex > 2){
           MGRPositionIndex = MGRPositionIndex%3;
        }
        MGRTargetVoltage = MGRCalcIntakePosition(MGRPositionIndex);
        //MGRTargetVoltage = MGRCalcLaunchPosition(MGRPositionIndex);
        mgrController.setSetpoint(MGRTargetVoltage);

        //axon.setTargetRotation(MGRTargetVoltage);
    }

    private double MGRCalcLaunchPosition(int position){
        double positions[] = {0.8, 1.915, 3.019};
        //double positions[] = {0.0, 120.0, 240.0};
        return positions[position];
    }

    public void MGRSetLaunchPosition(int index) {
        mgrMode = MGRMode.LAUNCH;
        MGRPositionIndex = index;
        MGRTargetVoltage = MGRCalcLaunchPosition(MGRPositionIndex);
        mgrController.setSetpoint(MGRTargetVoltage);
    }

    public void MGRNextLaunchPosition(){
        mgrMode = MGRMode.LAUNCH;
        MGRPositionIndex++;
        if(MGRPositionIndex > 2){
            MGRPositionIndex = MGRPositionIndex%3;
        }
        MGRTargetVoltage = MGRCalcLaunchPosition(MGRPositionIndex);
        mgrController.setSetpoint(MGRTargetVoltage);

        //axon.setTargetRotation(MGRTargetVoltage);
    }

    private double circleDiff(double a1, double a2) {
        if (a2 > a1 && a2 > 0 && a1 < 0 && Math.abs(a2 - a1) > Math.PI)
            return -((Math.PI - a2) + (a1 + Math.PI));
        else if (a1 > a2 && a1 > 0 && a2 < 0 && Math.abs(a2 - a1) > Math.PI)
            return (Math.PI - a1) + (a2 + Math.PI);
        else
            return a2 - a1;
    }

    private double MGRDiff(double a1, double a2) {
        final double C = 3.3/2;
        if (a2 > a1 && a2-a1 > C)       return a2 - 3.3 - a1;
        else if (a2 > a1 && a2-a1 <= C) return a2 - a1;
        else if (a1 > a2 && a1-a2 > C)  return 3.3 - a1 + a2;
        else if (a1 > a2 && a1-a2 <= C) return a2 - a1;
        else return 0;
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
    public void turnBy(double r){
        targetHeading+=r;
    }

    public void forwardBy(float inches){
        fwdMoving = true;
        targetDistance = inches;
        startTicks = -fr.getCurrentPosition();
    }

    public void strafeBy(float inches){
        strafeMoving = true;
        strafeTargetDistance = inches;
        startTicks = -br.getCurrentPosition();
    }

    public void startSpinny(){
        spinny.setPower(1);
    }

    public void stopSpinny(){
        spinny.setPower(0);
    }

    public void startShooter(){
        shooter.setPower(.4);
    }

    public void stopShooter(){
        shooter.setPower(0);
    }

    public void flick(){
        flicky.setPosition(1);
    }

    public void colorSensor(){
    // Get RGB values from the color sensor
    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();

    // Convert RGB to HSV
    float[] hsv = new float[3];
    RGBtoHSV(red, green, blue, hsv);

    // Extract Hue, Saturation, and Value
    float hue = hsv[0];
    float saturation = hsv[1];
    float value = hsv[2];

    // Add telemetry data to monitor RGB, HSV values
//        telemetry.addData("Red", red);
//        telemetry.addData("Green", green);
//        telemetry.addData("Blue", blue);
//        telemetry.addData("Hue", hue);
//        telemetry.addData("Saturation", saturation);
//        telemetry.addData("Value", value);

    // Adjusted color detection logic
    // Detect Green color (hue range 90 - 160, saturation > 0.4, value > 0.2)
        if (hue >= 90 && hue <= 160 && saturation > 0.4 && value > 0.2) {
        currentColor = "green";
    }
    // Detect Purple color (hue range 230 - 280, saturation > 0.4, value > 0.2)
        else if (hue >= 210 && hue <= 250 && saturation > 0.4 && value > 0.9) {
        currentColor = "purple";
    }
    // If no color is detected, mark as Unknown
        else {
        currentColor = "unknown";
    }

        //telemetry.addLine(axon.log());
}

    public void RGBtoHSV(int r, int g, int b, float[] hsv) {
        float max = Math.max(Math.max(r, g), b);
        float min = Math.min(Math.min(r, g), b);
        float delta = max - min;

        float h = 0;
        float s = (max == 0) ? 0 : (delta / max);
        float v = max / 255.0f;

        if (max == min) {
            h = 0;  // No hue
        } else {
            if (max == r) {
                h = (g - b) / delta;  // Red is max
            } else if (max == g) {
                h = (b - r) / delta + 2;  // Green is max
            } else {
                h = (r - g) / delta + 4;  // Blue is max
            }

            h *= 60;  // Convert to degrees
            if (h < 0) h += 360;  // Ensure positive hue
        }

        hsv[0] = h;  // Hue
        hsv[1] = s;  // Saturation
        hsv[2] = v;  // Value
    }

    public boolean isMoving(){
        return fwdMoving || turning || strafeMoving;
    }

    public void stop(){
        forward(0);
    }

    private void updateLimelight() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            limelightValid = true;
            Pose3D botpose = result.getBotpose_MT2();
            ta = result.getTa();
            tx = result.getTx();
            ty = result.getTy();
            bearingToTarget = Math.atan2(tx, ty) * 180.0 / Math.PI;
        }else{
            limelightValid = false;
        }
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