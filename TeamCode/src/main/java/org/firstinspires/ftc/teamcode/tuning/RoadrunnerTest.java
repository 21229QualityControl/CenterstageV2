package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(group = "Test")
public final class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 63, Math.toRadians(-90)));
        Intake intake = new Intake(hardwareMap);
        intake.initialize();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(24,  10), Math.toRadians(180))
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-55, 8), Math.toRadians(180))
                                .build(),
                        drive.actionBuilder(new Pose2d(-55, 8, Math.toRadians(180)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(180) - Math.PI)
                                .splineToConstantHeading(new Vector2d(24, 10), Math.toRadians(180) - Math.PI)
                                .splineToSplineHeading(new Pose2d(51, 21.5, Math.toRadians(200)), Math.toRadians(200) - Math.PI)
                                .build(),
                        drive.actionBuilder(new Pose2d(51, 21.5, Math.toRadians(200)))
                                .splineToSplineHeading(new Pose2d(24,  10, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-55, 8), Math.toRadians(180))
                                .build(),
                        drive.actionBuilder(new Pose2d(-55, 8, Math.toRadians(180)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(180) - Math.PI)
                                .splineToConstantHeading(new Vector2d(24, 10), Math.toRadians(180) - Math.PI)
                                .splineToSplineHeading(new Pose2d(53, 21.5, Math.toRadians(200)), Math.toRadians(200) - Math.PI)
                                .build()
                ));
    }
}
