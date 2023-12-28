package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.CameraProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.imgproc.Imgproc;
@Autonomous
// This uses the CameraProcessor
public class CameraProcessorTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Create an instance of the camera processor
        final CameraProcessor processor = new CameraProcessor();

        int w = 1920;
        int h = 1080;

        new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(w, h))
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();
        // Activate Bitmap consumers to ACTUALLY show the image on Dashboard. If the "max frames per second" variable is set to 0, there isn't a max limit and the camera can take infinite photos.
        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Position:", processor.position);
            telemetry.addData("Left Rectangle Saturation:", processor.getSatRectLeft());
            telemetry.addData("Center Rectangle Saturation:", processor.getSatRectCenter());
            telemetry.addData("Right Rectangle Saturation:", processor.getSatRectRight());
            telemetry.update();
            sleep(100);
        }
    }
}
