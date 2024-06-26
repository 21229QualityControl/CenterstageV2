package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.PartnerPreloadProcessor;
import org.firstinspires.ftc.teamcode.util.CameraUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "PartnerPreloadTest", group = "Test")
public class PartnerPreloadTest extends LinearOpMode {
    public static boolean RED = false;
    public static int SPIKE = 0; // 0 = right, 1 = middle, 2 = left
    public static boolean FALLBACK = false;
    public CameraUtil.DebugMode debugMode = CameraUtil.DebugMode.Dashboard;
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor aprilTag = PartnerPreloadProcessor.newAprilTagProcessor();
        PartnerPreloadProcessor processor = new PartnerPreloadProcessor(aprilTag);
        processor.setDebugMode(debugMode);

        VisionPortal portal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamOuttake"))
                .setCameraResolution(new Size(1280, 720)) // THIS CRASHES AT 1920x1080, GL ERROR: Out of memory!
                .addProcessor(aprilTag)
                .addProcessor(processor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            FtcDashboard.getInstance().startCameraStream(processor, 0);
        }

        while (opModeInInit()) {
            processor.updateTarget(SPIKE, RED);
            processor.fallback = FALLBACK;

            telemetry.addData("Red Side", RED);
            telemetry.addData("Spike Position", SPIKE);
            telemetry.addData("Target April Tag", processor.targetAprilTag);
            telemetry.addLine();

            telemetry.addData("April Tag Detected", processor.detecting);
            telemetry.addData("Left Average", processor.leftZoneAverage);
            telemetry.addData("Right Average", processor.rightZoneAverage);
            telemetry.addLine();

            telemetry.addData("Preload Left", processor.preloadLeft);

            if (processor.detecting) {
                telemetry.addData("x", processor.detectedPose.x);
                telemetry.addData("y", processor.detectedPose.y);
                telemetry.addData("z", processor.detectedPose.z);
                telemetry.addLine();
            }

            telemetry.update();
        }

        waitForStart();

        portal.stopStreaming();

        while (opModeIsActive()) {
            // the camera processor got result only when it is in active
            telemetry.addData("loop:", "loop");
            telemetry.update();
        }
    }
}
