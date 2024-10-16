package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ibby", group="TeleOp")
public class ibby extends LinearOpMode {
    //private Servo john;
    //private Servo joe;

    @Override
    public void runOpMode() throws InterruptedException {
        double joepos;
        double johnpos;
        
        Servo john = hardwareMap.get(Servo.class, "john");
        Servo joe = hardwareMap.get(Servo.class, "joe");
        waitForStart();

        while (opModeIsActive()) {
            joepos = (gamepad1.left_stick_x /2.0) +0.5;
            johnpos = (gamepad1.right_stick_x /2.0) +0.5;
            joe.setPosition(joepos);
            john.setPosition(johnpos);
            telemetry.addData("joe", "%.1f %.1f", gamepad1.left_stick_x,joepos);
            telemetry.update();

        }

    }
}