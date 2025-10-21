package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestTeleOp", group="TeleOp")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo MGR = hardwareMap.get(CRServo.class, "MGR");
        DcMotor spinny = hardwareMap.get(DcMotor.class, "spinny");
        Servo flicky = hardwareMap.get(Servo.class, "flicky");

        spinny.setDirection(DcMotorSimple.Direction.REVERSE);
        flicky.setPosition(1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                spinny.setPower(1);
            } else {
                spinny.setPower(0);
            }

            if (gamepad1.a) {
                MGR.setPower(1);
                MGR.setDirection(com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE);
            } else {
                MGR.setPower(0);
            }

            if (gamepad1.x) {
                flicky.setPosition(1);
                //servo.setDirection(com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE);
            } else {
                flicky.setPosition(0);
            }
        }
    }
}
