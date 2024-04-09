package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Autonomous(name = "Blue Right Auto 2+5", group = "Auto")
public class BlueRightAuto2_5 extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-48, 28, Math.toRadians(-90)),
            new Pose2d(-34, 20, Math.toRadians(-90)),
            new Pose2d(-36, 38, Math.toRadians(180))};
    public static Pose2d intermediate = new Pose2d(24,  11, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(-36, 11, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-57, 12, Math.toRadians(180));

    public static Pose2d secondStack = new Pose2d(-58, 18, Math.toRadians(150));
    public static Pose2d scoring = new Pose2d(56, 23, Math.toRadians(200));
    public static Pose2d park = new Pose2d(48, 24, Math.toRadians(180));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Right Auto 2+5");
    }

    @Override
    protected void onRun() {
        scorePreload();

        // First stack
        intakeStack(true);
        cycle(false);

        intakeStack(false);
        cycle(true);

        park();
    }

    private void scorePreload() {
        // Deliver spike
        if (SPIKE != 2) {
            sched.addAction(intake.wristDown());
        }
        TrajectoryActionBuilder bld = drive.actionBuilder(getStartPose())
                .strafeToLinearHeading(spike[SPIKE].position.plus(new Vector2d(0, -4)), spike[SPIKE].heading);
        if (SPIKE == 2) {
            bld = bld.strafeToLinearHeading(spike[SPIKE].position.plus(new Vector2d(6, -4)), spike[SPIKE].heading);
        }
        sched.addAction(bld.build());
        sched.addAction(intake.feedOpen());
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.1));
        sched.run();

        // Intake stack
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeBarelyOut()
                        ))
                        .afterDisp(8, new SequentialAction(
                                intake.feedClosed(),
                                new SleepAction(0.5),
                                intake.prepIntakeCount(true, true)

                        ))
                        .strafeToLinearHeading(stack.position, stack.heading)
                        .build()
        );
        sched.addAction(intake.intakeCount(false));
        sched.run();


        // Drive to preload
        boolean single = intake.pixelCount() != 2;
        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                .afterDisp(0, new SequentialAction(
                        intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                        intake.intakeOff()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI)
                .afterDisp(0, new ActionUtil.RunnableAction(() -> {
                    this.preloadProcessor.updateTarget(SPIKE, false);
                    this.preloadProcessor.detecting = false;
                    this.preloadProcessor.fallback = false;
                    this.preloadPortal.setProcessorEnabled(this.aprilTagProcessor, true);
                    this.preloadPortal.setProcessorEnabled(this.preloadProcessor, true);
                    this.preloadPortal.resumeStreaming();
                    this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                    return false;
                }))
                .afterDisp(1, new SequentialAction(
                        outtake.extendOuttakeCloseBlocking(),
                        outtake.armScoring(),
                        outtake.wristSideways(SPIKE == 2)
                ))
                .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading.toDouble() - Math.PI)
                        .build()
        );
        sched.run();

        // Detect spike
        long finalDetectTime = System.currentTimeMillis() + 500;
        sched.addAction(new SleepAction(0.2));
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            if (preloadProcessor.detecting) {
                Log.d("BACKDROP_PRELOADLEFT", String.valueOf(preloadProcessor.preloadLeft));
                this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                this.portal.stopStreaming();
                return false;
            } else if (System.currentTimeMillis() > finalDetectTime) {
                this.preloadProcessor.fallback = true;
            }
            return true;
        }));
        sched.run();

        // Score
        double off = 4;
        if (single) {
            off = 0;
            sched.addAction(outtake.wristVertical());
        } else if ((SPIKE == 0 && preloadProcessor.preloadLeft) || (SPIKE == 2 && !preloadProcessor.preloadLeft)) { // Sides
            off = 2.2;
        } else {
            sched.addAction(outtake.wristSideways(preloadProcessor.preloadLeft));
            sched.addAction(outtake.extendOuttakeCloseBlocking());
        }
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, preloadProcessor.preloadLeft ? -off : off)), AutoConstants.blueScoring[SPIKE].heading, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(12, new SequentialAction(
                        outtake.clawHalfOpen()
                ))
                .build()
        );
        sched.run();
    }

    private void intakeStack(boolean first) {
        TrajectoryActionBuilder bld = drive.actionBuilder(first ? new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading) : scoring)
                .afterDisp(1, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        outtake.clawOpen(),
                        outtake.retractOuttakeBlocking()
                ));
        if (first) {
            bld = bld.splineToConstantHeading(intermediate.position, intermediate.heading);
        } else {
            bld = bld.splineToSplineHeading(intermediate, intermediate.heading);
        }
        bld = bld.afterDisp(0, new SequentialAction(
                    intake.prepIntakeCount(!first, true), // One should be false here?
                    intake.intakeOn(),
                    outtake.extendOuttakeBarelyOut()
            ))
            .splineToConstantHeading(pastTruss.position, pastTruss.heading);
        if (first) {
            bld = bld.splineToConstantHeading(stack.position, stack.heading, drive.slowVelConstraint);
        } else {
            bld = bld.splineToSplineHeading(secondStack, secondStack.heading, drive.slowVelConstraint);
        }

        sched.addAction(bld.build());
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
        TrajectoryActionBuilder bld = drive.actionBuilder(second ? secondStack : stack)
                .setReversed(true);
        if (second) {
            bld = bld.splineToSplineHeading(pastTruss, pastTruss.heading.toDouble() - Math.PI);
        } else {
            bld = bld.splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI);
        }
        sched.addAction(bld
                .afterDisp(0, new SequentialAction(
                        intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                        intake.intakeOff()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeCycleBlocking(),
                                outtake.armScoring(),
                                intake.feedOpen()
                        ))
                .splineToSplineHeading(scoring, scoring.heading.toDouble() - Math.PI)
                .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed()); // Open & close feed in case outtake doesn't grab
        sched.run();
    }

    private void park() {
        sched.addAction(drive.actionBuilder(scoring)
                .strafeToLinearHeading(park.position, park.heading)
                .afterDisp(1, new SequentialAction(
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ))
                .build());
        sched.run();
    }
}
