package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Config
public abstract class AutoBase extends LinearOpMode {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;
    protected Vision vision;
    protected Plane plane;
    protected AutoActionScheduler sched;

    public static int SPIKE = 0;

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
        this.vision = new Vision(hardwareMap);
        this.plane = new Plane(hardwareMap);
        this.sched = new AutoActionScheduler(this::update);

        outtake.resetMotors();

        outtake.initialize();
        plane.initialize();

        onInit();

        while (opModeInInit()) {
            SPIKE = vision.objectPosition();
            vision.displayTelemetry(telemetry);

            printDescription();

            telemetry.addData("Spike Position", SPIKE);
            telemetry.update();
            idle();
        }

        // Auto start
        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence

        if (isStopRequested()) return; // exit if stopped

        onRun();
        drive.pose = getStartPose();
        sched.run();
        Memory.LAST_POSE = drive.pose;

        Log.d("Auto", "Auto ended at " + getRuntime());
    }

    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();
    protected void onInit() {}
    protected abstract void onRun();
}
