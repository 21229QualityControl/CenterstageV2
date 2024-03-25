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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(name = "PartnerPreloadTest", group = "Test")
public class PartnerPreloadTest extends LinearOpMode {
    public static boolean RED = false;
    public static int SPIKE = 0; // 0 = right, 1 = middle, 2 = left
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor aprilTag = PartnerPreloadProcessor.newAprilTagProcessor();
        PartnerPreloadProcessor processor = new PartnerPreloadProcessor(aprilTag);
        VisionPortal portal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamOuttake"))
                .setCameraResolution(new Size(1280, 720)) // THIS CRASHES AT 1920x1080, GL ERROR: Out of memory!
                .addProcessor(aprilTag)
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.FRONT)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            processor.updateTarget(SPIKE, RED);

            telemetry.addData("Red Side", RED);
            telemetry.addData("Spike Position", SPIKE);
            telemetry.addLine();

            telemetry.addData("April Tag Detected", processor.detecting);
            telemetry.addData("Left Average", processor.leftZoneAverage);
            telemetry.addData("Right Average", processor.rightZoneAverage);
            telemetry.addLine();

            telemetry.addData("Preload Left", processor.preloadLeft);

            telemetry.update();
        }
    }
}
