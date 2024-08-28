package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HydraTeleOp", group="PreSeason")
public class HydraTeleOp extends LinearOpMode {
    public DcMotor flMotor = null;
    public DcMotor frMotor = null;
    public DcMotor blMotor = null;
    public DcMotor brMotor = null;

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        double forward[] = {1, 1, 1, 1};
        double turn[] = {-1, 1, -1, 1};
        double strafe[] = {-1, 1, 1, -1};

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double p[] = {0, 0, 0, 0};
            for (int i = 0; i < 4; ++i) p[i] = forward[i] * gamepad1.left_stick_y;
            for (int i = 0; i < 4; ++i) p[i] += turn[i] * gamepad1.right_stick_x;
            for (int i = 0; i < 4; ++i) p[i] += strafe[i] * gamepad1.left_stick_x;

            flMotor.setPower(clamp(p[0], -1.0, 1.0));
            frMotor.setPower(clamp(p[1], -1.0, 1.0));
            blMotor.setPower(clamp(p[2], -1.0, 1.0));
            brMotor.setPower(clamp(p[3], -1.0, 1.0));

            //sleep(50);

            telemetry.addData("", "%.2f", p[0]);
            telemetry.addData("", "%.2f", p[1]);
            telemetry.addData("", "%.2f", p[2]);
            telemetry.addData("", "%.2f", p[3]);
            telemetry.update();

        }
    }
}
