package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

import java.util.Vector;

@Config
@Autonomous(name = "PlaneTest", group = "Auto")
public class AirplaneTest extends LinearOpMode {
    private Plane plane;
    private ActionScheduler sched;
    @Override
    public void runOpMode(){
        plane = new Plane(hardwareMap);
        sched = new ActionScheduler();
        plane.initialize();
        waitForStart();
        sched.queueAction(plane.scorePlane());
        new SleepAction(1);
    }

}
