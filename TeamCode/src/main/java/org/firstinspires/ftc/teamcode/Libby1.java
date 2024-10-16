package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name="Libby1", group="Autonomous")
public class Libby1 extends LinearOpMode {
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private static final boolean USE_WEBCAM = true;
    private void initTfod(HardwareMap hardwareMap) {
        String fileName = "TeamElementBlue.tflite";
       // if (color.equals(ArenaColor.Red)) {
            //fileName = "TeamElement_Red.tflite";
       // }
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(fileName)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }
        @Override
    public void runOpMode(){





        //initTfod(hardwareMap);
        waitForStart();

            //TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.61, -58.98, Math.toRadians(90.00)))
            //        .splineTo(new Vector2d(-28.78, 2.36), Math.toRadians(123.36))
            //        .build();

            MecanumDrive drive =new MecanumDrive(hardwareMap,new Pose2d(11.61,-58.98,Math.toRadians(90)));
        Action traj1;
        traj1=drive.actionBuilder(drive.pose)
                //.setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-28.78, 2.36), Math.toRadians(123.36))

                //.turn(Math.toRadians(180))
                //.lineToY(50)
                //.turn(Math.toRadians(90))
                //.lineToX(-50)
               //.splineTo(new Vector2d(0, 75), Math.toRadians(90.0))
                //.splineTo(new Vector2d(-40, 0), Math.toRadians(90))
                //.splineTo(new Vector2d(0, -60), Math.toRadians(90))
                //.splineTo(new Vector2d(40, 0), Math.toRadians(90))
                //.splineTo(new Vector2d(12.74, 18.59), Math.toRadians(111.12))
                .build();

        Actions.runBlocking(new SequentialAction(traj1));

        while (!isStopRequested() && !opModeIsActive()) {
            sleep(50);
        }

        if (isStopRequested()) return;

}
}

