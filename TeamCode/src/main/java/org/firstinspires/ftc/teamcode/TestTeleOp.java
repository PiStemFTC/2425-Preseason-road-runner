package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestTeleOp", group="TeleOp")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo servo = hardwareMap.get(CRServo.class, "servo");
        DcMotor lSpinny = hardwareMap.get(DcMotor.class, "lSpinny");
        DcMotor rSpinny = hardwareMap.get(DcMotor.class, "rSpinny");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        rSpinny.setDirection(DcMotorSimple.Direction.REVERSE);
        lSpinny.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive())
            if (gamepad1.left_bumper) {
                lSpinny.setPower(1);
                rSpinny.setPower(1);
            } else {
                lSpinny.setPower(0);
                rSpinny.setPower(0);
            }

        if(gamepad1.right_bumper){
            intake.setPower(1);
        } else{
            intake.setPower(0);
        }

        if (gamepad1.a) {
            servo.setPower(1);
            servo.setDirection(com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE);
        } else {
            servo.setPower(0);
        }
    }
}
