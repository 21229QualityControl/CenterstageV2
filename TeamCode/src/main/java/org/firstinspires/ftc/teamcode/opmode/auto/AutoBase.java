package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraProcessor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.FrontSensors;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public abstract class AutoBase extends LinearOpMode {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
    protected FrontSensors frontSensors;
    protected Plane plane;
    protected AutoActionScheduler sched;

    protected CameraProcessor processor;
    protected VisionPortal portal;

    protected int w = 1920;
    protected int h = 1080;

    public static int SPIKE = -1;

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }

    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        Memory.LAST_POSE = getStartPose();
        Memory.RAN_AUTO = true;

        // Init subsystems
        this.drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        this.intake = new Intake(hardwareMap);
        this.outtake = new Outtake(hardwareMap);
         this.frontSensors = new FrontSensors(hardwareMap);
        this.plane = new Plane(hardwareMap);
        this.sched = new AutoActionScheduler(this::update);
        this.processor = new CameraProcessor();
        this.portal = new VisionPortal.Builder()
                // Get the actual camera on the robot, add the processor, state the orientation of the camera.
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(w, h))
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        outtake.resetMotors();

        outtake.initialize();
        plane.initialize();
        intake.initialize();

        onInit();

        SPIKE = -1;
        while (opModeInInit()) {
            // Below is the code for the distance sensors
            // SPIKE = vision.objectPosition();
            // Below is the code for the camera vision

            int cameraReading = processor.position;
            int sensorReading = frontSensors.rawObjectPosition();
            if (cameraReading > -1) {
                SPIKE = cameraReading;
                telemetry.addData("Camera", "Working - " + cameraReading);
                telemetry.addData("Sensor", "Unused - " + sensorReading);
            } else if (cameraReading == -1) {
                telemetry.addData("Camera", "Not working - " + cameraReading);
                if (sensorReading > -1) {
                    SPIKE = sensorReading;
                    telemetry.addData("Backup Sensor", "Working - " + sensorReading);
                } else {
                    SPIKE = -1;
                    telemetry.addData("Backup Sensor", "Not working - " + sensorReading);
                }
            }

            telemetry.addData("Position:", processor.position);
            telemetry.addData("Left Rectangle Saturation:", "%.3f", processor.getSatRectLeft());
            telemetry.addData("Center Rectangle Saturation:", "%.3f", processor.getSatRectCenter());
            telemetry.addData("Right Rectangle Saturation:", "%.3f", processor.getSatRectRight());
            telemetry.update();

//            vision.displayTelemetry(telemetry);
//            printDescription();

            idle();
        }

        if (SPIKE == -1) {
            SPIKE = 0;
        }
        portal.stopStreaming();

        // Auto start
        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence

        if (isStopRequested()) return; // exit if stopped

        drive.updatePoseEstimate();
        drive.pose = getStartPose();
        onRun();
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
