package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.CameraProcessor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.vision.PartnerPreloadProcessor;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.util.LED;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public abstract class AutoBase extends LinearOpMode {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
    protected Plane plane;
    protected AutoActionScheduler sched;
    protected CameraProcessor processor;
    protected AprilTagProcessor aprilTagProcessor;
    protected PartnerPreloadProcessor preloadProcessor;
    protected VisionPortal portal;
    protected VisionPortal preloadPortal;
    protected LED led;

    protected int w = 640;
    protected int h = 480;

    public static int SPIKE = -1;

    final public void update() {
        int pixelCount = intake.pixelCount();
        telemetry.addData("Time left", 30 - getRuntime());
        telemetry.addData("Pixel Count", pixelCount);
        telemetry.addData("Number Intaked", intake.numIntaked);
        telemetry.addData("Intake Reversing", intake.isReversing());
        if (pixelCount == 0) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (pixelCount == 1) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (pixelCount == 2) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        outtake.update();
        intake.update();
        telemetry.update();
    }

    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing");
        telemetry.update();

        Memory.LAST_POSE = getStartPose();
        Memory.RAN_AUTO = true;

        // Init subsystems
        this.drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        this.intake = new Intake(hardwareMap);
        this.outtake = new Outtake(hardwareMap);
        this.plane = new Plane(hardwareMap);
        this.sched = new AutoActionScheduler(this::update);
        this.processor = new CameraProcessor();
        this.aprilTagProcessor = PartnerPreloadProcessor.newAprilTagProcessor();
        this.preloadProcessor = new PartnerPreloadProcessor(aprilTagProcessor);
        this.led = new LED(hardwareMap);

        int[] visionPortalViewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        this.portal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(w, h))
                .addProcessor(processor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(visionPortalViewIDs[0])
                .build();
        this.preloadPortal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamOuttake"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessors(aprilTagProcessor, preloadProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(visionPortalViewIDs[1])
                .build();


        preloadPortal.setProcessorEnabled(aprilTagProcessor, false);
        preloadPortal.setProcessorEnabled(preloadProcessor, false);

        outtake.initialize(false);
        plane.initialize();
        intake.initialize();

        telemetry.addLine("Initializing outtake slides");
        telemetry.update();
        outtake.prepInitializeSlides();
        sleep(200);
        while (opModeInInit() && outtake.initializeSlides()) {}

        telemetry.addLine("Initializing cameras");
        telemetry.update();
        while (opModeInInit() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING || preloadPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {}

        onInit();

        SPIKE = -1;
        while (opModeInInit()) {
            int cameraReading = processor.position;
            if (cameraReading > -1) {
                SPIKE = cameraReading;
                telemetry.addData("Camera", "Working - " + cameraReading);
            } else if (cameraReading == -1) {
                telemetry.addData("Camera", "Not working - " + cameraReading);
            }

            telemetry.addData("Position:", processor.position);
            telemetry.addData("Left Rectangle Saturation:", "%.3f", processor.getSatRectLeft());
            telemetry.addData("Center Rectangle Saturation:", "%.3f", processor.getSatRectCenter());
            telemetry.addData("Right Rectangle Saturation:", "%.3f", processor.getSatRectRight());
            telemetry.update();

            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

            outtake.update();
            intake.update();

            idle();
        }

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        if (SPIKE == -1) {
            SPIKE = 0;
        }
        portal.stopStreaming();
        preloadPortal.stopStreaming();
        portal.setProcessorEnabled(processor, false);

        // Auto start
        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence

        if (isStopRequested()) return; // exit if stopped

        drive.updatePoseEstimate();
        drive.pose = getStartPose();
        Memory.FINISHED_AUTO = false;
        onRun();
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            Memory.FINISHED_AUTO = true;
            return false;
        }));
        drive.pose = getStartPose();
        //drive.imu.resetYaw();

        sched.run();
        Memory.LAST_POSE = drive.pose;

        Log.d("Auto", "Auto ended at " + getRuntime());
    }

    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();
    protected void onInit() {}
    protected abstract void onRun();
}
