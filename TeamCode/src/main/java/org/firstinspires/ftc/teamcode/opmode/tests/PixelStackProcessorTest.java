package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.PixelStackProcessor;
import org.firstinspires.ftc.teamcode.util.CameraUtil;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PixelStackProcessorTest extends LinearOpMode {
    public CameraUtil.DebugMode debugMode = CameraUtil.DebugMode.Dashboard;
    public void runOpMode() throws InterruptedException {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        // Create an instance of the camera processor
        final PixelStackProcessor processor = new PixelStackProcessor();
        processor.setDebugMode(debugMode);

        int w = 640;
        int h = 480;

        VisionPortal portal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(w, h))
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        // Activate Bitmap consumers to ACTUALLY show the image on Dashboard. If the "max frames per second" variable is set to 0, there isn't a max limit and the camera can take infinite photos.
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            FtcDashboard.getInstance().startCameraStream(processor, 0);
        }

        while (opModeInInit()) {
            // the camera processor got result only when it is in active
            telemetry.addData("Init", "init");
            telemetry.update();

            sleep(100);
        }

        waitForStart();

        while (opModeIsActive()) {
            // the camera processor got result only when it is in active
            telemetry.addData("loop:", "loop");
            telemetry.update();
        }
    }
}
