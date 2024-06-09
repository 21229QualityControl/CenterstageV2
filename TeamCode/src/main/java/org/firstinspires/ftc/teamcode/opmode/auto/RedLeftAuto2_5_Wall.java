package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitPath;
import org.firstinspires.ftc.teamcode.pathing.WaitPositionCommand;
import org.firstinspires.ftc.teamcode.pathing.Waypoint;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

@Autonomous(name = "Red Left Auto 2+5 Wall", group = "Auto")
public class RedLeftAuto2_5_Wall extends AutoBase {
    public static Waypoint start = new Waypoint(new Pose(-36, -63, Math.toRadians(90)), 15);
    public static Waypoint[] spike = {
            new Waypoint(new Pose(-33, -37, Math.toRadians(45)), 15),
            new Waypoint(new Pose(-48, -30, Math.toRadians(45)), 15),
            new Waypoint(new Pose(-48, -39, Math.toRadians(90)), 15)
    };
    public static Waypoint spikeBackedOut = new Waypoint(new Pose(-48, 50, Math.toRadians(-90)), 15);
    public static Waypoint[] vision = {
            new Waypoint(new Pose(42, -28, Math.toRadians(-180)), 15),
            new Waypoint(new Pose(42, -34.5, Math.toRadians(-180)), 15),
            new Waypoint(new Pose(42, -40, Math.toRadians(-180)), 15)
    };
    public static Waypoint intermediate = new Waypoint( new Pose(-36, -60, Math.toRadians(-180)), 20);
    public static Waypoint intermediateAfterPreload = new Waypoint( new Pose(-40, -61, Math.toRadians(-180)), 15);
    public static Waypoint pastTruss = new Waypoint(new Pose(30, -60, Math.toRadians(-180)), 20);
    public static Waypoint stackAfterPreload = new Waypoint(new Pose(-61, -42, Math.toRadians(-210)), 15);
    public static Waypoint stack = new Waypoint(new Pose(-60, -37, Math.toRadians(-205)), 15);
    public static Waypoint stackLast = new Waypoint(new Pose(-60, -26, Math.toRadians(-205)), 15);
    public static Waypoint park = new Waypoint(new Pose(44, -46, Math.toRadians(-180)), 15);
    public static Waypoint scoring = new Waypoint(new Pose(53, -43, Math.toRadians(-160)), 15);
    private Waypoint backdrop(int pos, double off) {
        Waypoint[] backdrop = {
                new Waypoint(new Pose(54, -28 + off, Math.toRadians(-180)), 15),
                new Waypoint(new Pose(54, -34.5 + off, Math.toRadians(-180)), 15),
                new Waypoint(new Pose(54, -43 + off, Math.toRadians(-180)), 15)
        };
        return backdrop[pos];
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(start.getPoint().x, start.getPoint().y, ((Pose)start.getPoint()).heading);
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Red Left Auto 2+5 Wall");
    }

    @Override
    protected void onRun() {
        firstCycle();

        intakeStack(true);
        cycle(true);

        intakeStack(false);
        cycle(false);

        park();
    }

    double off;
    private void firstCycle() {
        // Deliver spike
        sched.addAction(intake.wristPreload());
        sched.addAction(new SleepAction(0.4));
        sched.addAction(
                SPIKE == 2 ? new PurePursuitCommand(drive, new PurePursuitPath(
                        start,
                        new Waypoint(new Pose(-33, -41, Math.toRadians(90)), 15),
                        spike[SPIKE]
                )) : new PurePursuitCommand(drive, new PurePursuitPath(
                        start,
                        spike[SPIKE]
                ))
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.1));
        sched.run();

        // Intake stack
        if (SPIKE == 0) {
            sched.addAction(new PurePursuitCommand(drive, new PurePursuitPath(
                    spike[SPIKE],
                    spikeBackedOut
            )));
        }
        sched.addAction(
                new ParallelAction(
                        new PurePursuitCommand(drive, new PurePursuitPath(
                                SPIKE == 0 ? spikeBackedOut : spike[SPIKE],
                                SPIKE != 1 ? stack : stackAfterPreload
                        )),
                        new SequentialAction(
                                outtake.extendOuttakeBarelyOut(),
                                intake.prepIntakeCount(true, false)
                        )
                )
        );
        sched.addAction(intake.intakeCount(true));
        sched.run();
        Log.d("SPIKE:", Integer.toString(SPIKE));
        // Drive to preload
        boolean single = intake.pixelCount() != 2;
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        SPIKE != 1 ? stack : stackAfterPreload,
                        intermediateAfterPreload,
                        pastTruss,
                        vision[SPIKE]
                )),
                new SequentialAction(
                        outtake.retractOuttakeBlocking(),
                        new WaitPositionCommand(drive, -36, true, true), // intermediate
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed(),
                        new WaitPositionCommand(drive, 30, true, true), // pastTruss
                        new ActionUtil.RunnableAction(() -> {
                            this.preloadProcessor.updateTarget(SPIKE, false);
                            this.preloadProcessor.detecting = false;
                            this.preloadProcessor.fallback = false;
                            this.preloadPortal.setProcessorEnabled(this.aprilTagProcessor, true);
                            this.preloadPortal.setProcessorEnabled(this.preloadProcessor, true);
                            this.preloadPortal.resumeStreaming();
                            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                            return false;
                        }),
                        new SequentialAction(
                                intake.intakeOff(),
                                outtake.extendOuttakePartnerBlocking(),
                                outtake.armScoring(),
                                outtake.wristVerticalFlip()
                        ))
                )
        );
        sched.run();

        // Detect spike
        long finalDetectTime = System.currentTimeMillis() + 500;
        sched.addAction(new SleepAction(0.2));
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            if (preloadProcessor.detecting) {
                Log.d("BACKDROP_PRELOADLEFT", String.valueOf(preloadProcessor.preloadLeft));
                this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                saveImage(this.preloadPortal);
                this.preloadPortal.stopStreaming();
                return false;
            } else if (System.currentTimeMillis() > finalDetectTime) {
                this.preloadProcessor.fallback = true;
                Log.d("BACKDROP_PRELOADLEFT", "FALLBACK");
            }
            return true;
        }));
        sched.run();

        // Score
        off = 4;
        if (single) {
            off = 0;
            sched.addAction(outtake.wristVertical());
        } else if ((SPIKE == 0 && preloadProcessor.preloadLeft) || (SPIKE == 2 && !preloadProcessor.preloadLeft)) { // Sides
            off = 2.5;
        } else {
            sched.addAction(outtake.wristSideways(preloadProcessor.preloadLeft));
        }
        if (preloadProcessor.preloadLeft) {
            off *= -1;
        }
        sched.addAction(new PurePursuitCommand(drive, new PurePursuitPath(
                vision[SPIKE],
                backdrop(SPIKE, off)
        )));
        sched.addAction(outtake.clawHalfOpen());
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        backdrop(SPIKE, off),
                        pastTruss,
                        intermediate,
                        first ? stack : stackLast
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, 48, false, true), // Back out from backdrop
                        new SequentialAction(
                                outtake.wristVertical(),
                                outtake.armStored(),
                                intake.feedClosed(),
                                new SleepAction(0.3),
                                outtake.clawOpen(),
                                outtake.retractOuttakeBlocking()
                        ),
                        new WaitPositionCommand(drive, -36, false, true), // intermediate
                        intake.prepIntakeCount(false, false)
                )
        ));
        sched.run();


        sched.addAction(intake.intakeCount(true));
        sched.run();
    }

    private void cycle(boolean first) {
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        first ? stack : stackLast,
                        intermediate,
                        pastTruss,
                        scoring
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, -36, true, true), // intermediate
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed(),
                        new WaitPositionCommand(drive, 10, true, true), // pastTruss
                        intake.intakeOff(),
                        outtake.extendOuttakeCycleBlocking(),
                        outtake.armScoring(),
                        intake.feedOpen()
                )
        ));
        sched.run();

        // Score
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed());
        sched.addAction(new SleepAction(0.2));
        sched.run();
    }

    private void park() {
        sched.addAction(new ParallelAction(
                new PurePursuitCommand(drive, new PurePursuitPath(
                        scoring,
                        park
                )),
                new SequentialAction(
                        new WaitPositionCommand(drive, 48, false, true), // Back out from backdrop
                        outtake.armStored(),
                        outtake.clawOpen(),
                        intake.feedClosed(),
                        outtake.retractOuttakeBlocking()
                )
        ));
        sched.run();
    }
}
