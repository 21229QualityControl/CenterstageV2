package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "Test")
@Config
public final class ManualAxialTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        Pose2d start = new Pose2d(0, 0, 0);
        drive.pose = start;

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(start)
                            .strafeTo(start.position.plus(new Vector2d(DISTANCE, 0)))
                            .strafeTo(start.position)
                            .build());
        }
    }
}
