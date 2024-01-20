package org.firstinspires.ftc.teamcode.opmode.tests;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
//@Disabled
@TeleOp(group = "Test")
public class AprilTagTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        processor = new AprilTagProcessor.Builder().build();

        int w = 640;
        int h = 480;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(w, h))
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detected = processor.getDetections();
            telemetry.addLine(String.format("\n==== Detected %d", detected.size()));
            for(AprilTagDetection d: detected) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", d.id, d.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", d.ftcPose.x,
                        d.ftcPose.y, d.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", d.ftcPose.pitch,
                        d.ftcPose.roll, d.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", d.ftcPose.range,
                        d.ftcPose.bearing, d.ftcPose.elevation));
            }
            telemetry.update();
            sleep(3000);
        }
    }
}
