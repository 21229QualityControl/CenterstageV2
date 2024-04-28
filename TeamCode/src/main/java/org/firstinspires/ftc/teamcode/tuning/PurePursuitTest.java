package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(group = "Test")
public final class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 63, Math.toRadians(-90)));
        Intake intake = new Intake(hardwareMap);
        intake.initialize();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new PurePursuitCommand(drive, new PurePursuitPath(
                                new Waypoint(new Pose(12, 63, Math.toRadians(-90)), 15),

                                // Drive to stack
                                new Waypoint(new Pose(24,  10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-36, 10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-55, 8, Math.toRadians(180)), 15)
                        )),
                        new PurePursuitCommand(drive, new PurePursuitPath(
                                // Score
                                new Waypoint(new Pose(-55, 8, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-36, 10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(24,  10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(51, 21.5, Math.toRadians(200)), 15)
                        )),
                        new PurePursuitCommand(drive, new PurePursuitPath(
                                // Drive to stack
                                new Waypoint(new Pose(51, 21.5, Math.toRadians(200)), 15),
                                new Waypoint(new Pose(24,  10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-36, 10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-55, 8, Math.toRadians(180)), 15)
                        )),
                        new PurePursuitCommand(drive, new PurePursuitPath(
                                // Score
                                new Waypoint(new Pose(-55, 8, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(-36, 10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(24,  10, Math.toRadians(180)), 15),
                                new Waypoint(new Pose(51, 21.5, Math.toRadians(200)), 15)
                        ))
                ));
    }
}
