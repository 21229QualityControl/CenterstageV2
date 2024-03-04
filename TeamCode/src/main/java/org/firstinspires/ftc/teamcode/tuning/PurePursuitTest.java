package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
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

@TeleOp(group = "Test")
public final class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        Actions.runBlocking(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        new Waypoint(new Pose(0, 0, 0), 10),
                        new Waypoint(new Pose(0, 10, 0), 10),
                        new Waypoint(new Pose(10, 10, 0), 10)
                )));
    }
}
