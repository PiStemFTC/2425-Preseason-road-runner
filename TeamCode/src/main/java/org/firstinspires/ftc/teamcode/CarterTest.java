package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo;
import org.firstinspires.ftc.teamcode.RTPAxon;



@TeleOp(name="CarterTest", group="TeleOp")
    public class CarterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        CRServo zoomServo = hardwareMap.get(CRServo.class, "zoomServo");
        CRServo servo = hardwareMap.get(CRServo.class, "zoomServo");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "analogInput");
        RTPAxon axon = new RTPAxon(servo, encoder);
        axon.setMaxPower(0.5);  // Limit max power to 50%
        axon.setPidCoeffs(0.02, 0.0005, 0.0025);  // Set PID coefficients

        DcMotor motors[] = {
                fl, fr,
                bl, br
        };



        float direction [] = {
                1.0f,-1.0f,
                1.0f,-1.0f
        };

        float turn [] = {
                -1.0f, 1.0f,
                -1.0f, 1.0f
        };

        float strafe [] = {
                -1.0f, -1.0f,
                1.0f, 1.0f
        };

        waitForStart();
        axon.setTargetRotation(90);    // Move to 90 degrees absolute
        while(opModeIsActive()){
            float y = gamepad1.left_stick_y;
            float x = gamepad1.right_stick_x;
            float s = gamepad1.left_stick_x;
            for (int i = 0; i<4; ++i){
                motors[i].setPower(y*direction[i] + x*direction[i]*turn[i] + s*direction[i]*strafe[i]);
            }
            axon.update(); // Must be called every loop
            //axon.changeTargetRotation(45); // Move 45 degrees from current position



        }
    }
}
