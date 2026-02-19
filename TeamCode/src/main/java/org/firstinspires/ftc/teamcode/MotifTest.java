package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Motif Test", group="TeleOp")
public class MotifTest extends LinearOpMode {

    class Motif {
        String order = "gpp";
        int motifIndex = -1;

        String[] chambers = {"", "", ""};



        boolean isLoaded() {
            for (int i = 0; i < 3; ++i) {
                if (chambers[i].isEmpty() || chambers[i].equals("EMPTY")) {
                    return false;
                }
            }
            return true;
        }
        public char color = ' ';
        int selectNext() {
            motifIndex++;
            if (motifIndex > 2)
                motifIndex = 0;
            color = order.charAt(motifIndex);
            for(int i = 0; i < 3; i++){
                if(color == 'p' && chambers[i].equals("PURPLE")){
                    chambers[i] = "";
                    return i;
                } else if(color == 'g' && chambers[i].equals("GREEN")){
                    chambers[i] = "";
                    return i;
                }
            }

            return 0;
        }
    }
    Motif motif = new Motif();

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isLoaded = false;
        Bessie bessie = new Bessie(telemetry);
        bessie.initializeHardware(hardwareMap);
        bessie.webcam(hardwareMap);

        bessie.limelight.pipelineSwitch(1);
        bessie.limelight.start();

        bessie.MGRNextIntakePosition();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpadUpWasPressed()) {
                //bessie.MGRNextLaunchPosition();
                bessie.MGRSetLaunchPosition(motif.selectNext());
            }
            if (gamepad2.xWasPressed()) {
                motif.chambers = bessie.getChambers();
            }

            if (gamepad2.yWasPressed()){
                motif.motifIndex = -1;
                bessie.MGRSetLaunchPosition(0);
            }
            bessie.updateMGR();

            if (!motif.isLoaded() && !isLoaded) {
                motif.chambers = bessie.getChambers();
                if (motif.isLoaded())
                    isLoaded = true;
            }

            telemetry.addData("motif", motif.order);
            telemetry.addLine(motif.chambers[0] + "," + motif.chambers[1] + "," + motif.chambers[2]);
            telemetry.addData("motifIndex", motif.motifIndex);
            telemetry.addData("mgrIndex", bessie.MGRPositionIndex);
            telemetry.addData("currentColor", motif.color);
            telemetry.update();
        }

    }
}

