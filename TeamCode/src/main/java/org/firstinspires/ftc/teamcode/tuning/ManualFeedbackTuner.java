package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "Test")
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        drive.pose =  new Pose2d(0, 0, 0);

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
        }
    }
}
