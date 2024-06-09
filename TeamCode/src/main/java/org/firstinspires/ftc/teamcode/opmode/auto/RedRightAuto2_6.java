package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.WaitPositionCommand;
import org.firstinspires.ftc.teamcode.pathing.Waypoint;

@Autonomous(name = "Red Right Auto 2+6", group = "Auto")
public class RedRightAuto2_6 extends AutoBase {
    public static Waypoint start = new Waypoint(new Pose(12, -63, Math.toRadians(90)), 15);
    public static Waypoint[] spike = {
            new Waypoint(new Pose(34, -33, Math.toRadians(-180)), 15),
            new Waypoint(new Pose(28, -23, Math.toRadians(-180)), 15),
            new Waypoint(new Pose(9, -34, Math.toRadians(135)), 15)
    };
    public static Waypoint[] backdrop = {
            new Waypoint(new Pose(54, -28, Math.toRadians(180)), 15),
            new Waypoint(new Pose(54, -34.5, Math.toRadians(180)), 15),
            new Waypoint(new Pose(54, -43, Math.toRadians(180)), 15)
    };
    public static Waypoint intermediate = new Waypoint(new Pose(30,  -7, Math.toRadians(-180)), 20);
    public static Waypoint pastTruss = new Waypoint(new Pose(-36, -7, Math.toRadians(-180)), 20);
    public static Waypoint stack = new Waypoint(new Pose(-55.5, -13, Math.toRadians(-180)), 15);
    public static Waypoint secondStack = new Waypoint(new Pose(-59, -21, Math.toRadians(-150)), 15);
    public static Waypoint scoring = new Waypoint(new Pose(54.5, -22, Math.toRadians(-200)), 15);
    public static Waypoint park = new Waypoint(new Pose(44, -20, Math.toRadians(-180)), 15);

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(start.getPoint().x, start.getPoint().y, ((Pose)start.getPoint()).heading);
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Red Right Auto 2+6");
    }

    @Override
    protected void onInit() {
        sched.addAction(outtake.clawSingleClosed());
        sched.run();
    }

    @Override
    protected void onRun() {
        scorePreload();

        // First stack
        intakeStack(true, false);
        cycle(false);

        intakeStack(false, false);
        cycle(false);

        // Second stack (need 8 seconds)
        if (getRuntime() < 22) {
            intakeStack(false, true);
            cycle(true);
        }

        /*intakeStack(false, true);
        cycle(true);*/

        park();
    }

    private void scorePreload() {
        // Deliver spike
        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());
        sched.addAction(
                SPIKE != 2 ? new PurePursuitCommand(drive, new PurePursuitPath(
                        start,
                        new Waypoint(new Pose(28, -45, ((Pose)spike[SPIKE].getPoint()).heading), 20),
                        spike[SPIKE]
                )) : new PurePursuitCommand(drive, new PurePursuitPath(
                        start,
                        spike[SPIKE]
                ))
        );
        sched.addAction(intake.wristStored());
        sched.run();

        // Score preload
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        spike[SPIKE],
                        backdrop[SPIKE]
                )),
                new SequentialAction(
                        outtake.extendOuttakeCloseBlocking(),
                        outtake.armScoring(),
                        outtake.wristVerticalFlip()
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.run();
    }

    private void intakeStack(boolean first, boolean nextStack) {
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        first ? backdrop[SPIKE] : scoring,
                        intermediate,
                        pastTruss,
                        nextStack ? secondStack : stack
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, 48, false, true), // Away from backdrop
                        new SequentialAction(
                                outtake.wristVertical(),
                                outtake.armStored(),
                                outtake.retractOuttakeBlocking()
                        ),
                        new WaitPositionCommand(drive, 24, false, true), // intermediate
                        new SequentialAction(
                                intake.prepIntakeCount(first || nextStack, false),
                                nextStack ? intake.intakeOn() : intake.intakeSlow(),
                                outtake.extendOuttakeBarelyOut()
                        )
                )
        ));
        if (first) { // Score out of the way
            if (SPIKE == 0) {
                SPIKE = 1;
            } else {
                SPIKE = 0;
            }
        }
        sched.addAction(new SequentialAction(
                intake.intakeCount(false),
                outtake.retractOuttake()
        ));
        sched.run();
    }

    private void cycle(boolean second) {
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        second ? secondStack : stack,
                        pastTruss,
                        intermediate,
                        scoring
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, -36, true, true), // pastTruss
                        new SequentialAction(
                                intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                                intake.intakeOff()
                        ),
                        new WaitPositionCommand(drive, 21, true, true), // intermediate
                        new SequentialAction(
                                second ? outtake.extendOuttakeCycleHighBlocking() : outtake.extendOuttakeCycleBlocking(),
                                outtake.armScoring(),
                                intake.feedOpen()
                        )
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed()); // Open & close feed in case outtake doesn't grab
        sched.run();
    }

    private void park() {
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        scoring,
                        park
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, 50, false, true),
                        new SequentialAction(
                                outtake.armStored(),
                                outtake.retractOuttakeBlocking()
                        )
                )
        ));
        sched.run();
    }
}
