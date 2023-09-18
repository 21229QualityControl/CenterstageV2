package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public abstract class AutoBase extends LinearOpMode {
    protected MecanumDrive drive;
    protected Outtake outtake;
    protected Intake intake;

    final public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing... Please wait");
        telemetry.update();

        Memory.LAST_POSE = getStartPose();
        Memory.RAN_AUTO = true;

        // Init subsystems
        this.drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
        this.intake = new Intake(hardwareMap);
        this.outtake = new Outtake(hardwareMap);

        onInit();

        while (opModeInInit()) {
            // Update vision

            printDescription();
            // Print vision status

            telemetry.update();
            idle();
        }

        // Auto start
        resetRuntime(); // reset runtime timer
        Memory.saveStringToFile(String.valueOf(System.currentTimeMillis()), Memory.SAVED_TIME_FILE_NAME); // save auto time for persistence
        // TODO: Stop streaming vision

        if (isStopRequested()) return; // exit if stopped

        onRun();

        Log.d("Auto", "Auto ended at " + getRuntime());
    }

    protected abstract Pose2d getStartPose();
    protected abstract void printDescription();
    protected void onInit() {}
    protected abstract void onRun();
}
