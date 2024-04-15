package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Autonomous(name = "Blue Right Auto 2+3", group = "Auto")
public class BlueRightAuto2_3 extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-46, 44, Math.toRadians(-90)),
            new Pose2d(-48, 36, Math.toRadians(-45)),
            new Pose2d(-33, 41, Math.toRadians(-45))};
    public static Pose2d intermediate = new Pose2d(-36, 60, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(40, 60, Math.toRadians(180));
    public static Pose2d stackAfterPreload = new Pose2d(-59, 44, Math.toRadians(210));
    public static Pose2d stack = new Pose2d(-59, 39, Math.toRadians(205));
    public static Pose2d park = new Pose2d(48, 42, Math.toRadians(180));
    public static Pose2d detectPartner = new Pose2d(48, 60, Math.toRadians(180));
    public static Pose2d scoring = new Pose2d(55, 42, Math.toRadians(180));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Right Auto 2+3");
    }

    @Override
    protected void onRun() {
        firstCycle();

        intakeStack();
        cycle();
    }

    double off;
    private void firstCycle() {
        // Deliver spike
        sched.addAction(intake.wristPreload());
        sched.addAction(new SleepAction(0.4));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToSplineHeading(spike[SPIKE].position.plus(new Vector2d(0, -4)), spike[SPIKE].heading) // It doesn't go far enough? Perhaps roadrunner needs to be tuned better
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.1));
        sched.run();

        // Intake stack
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeBarelyOut(),
                                intake.prepIntakeCount(true, true)
                        ))
                        .strafeToLinearHeading(stackAfterPreload.position, stackAfterPreload.heading)
                        .build()
        );
        sched.addAction(intake.intakeCount(true));
        sched.run();

        if (intake.pixelCount() == 0) {
            tryAgain(stackAfterPreload, true, true);
        }

        // Drive to preload
        boolean single = intake.pixelCount() != 2;
        sched.addAction(
                drive.actionBuilder(stackAfterPreload)
                        .setReversed(true)
                        .afterDisp(0, outtake.retractOuttakeBlocking())
                        .splineToSplineHeading(intermediate, intermediate.heading.toDouble() - Math.PI, drive.slowestVelConstraint, drive.slowestAccelConstraint)
                        .afterDisp(0, intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed())
                        .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
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
                                intake.intakeOff(),
                                outtake.extendOuttakePartnerBlocking(),
                                outtake.armScoring(),
                                outtake.wristVerticalFlip()
                        ))
                        .splineToConstantHeading(detectPartner.position, detectPartner.heading.toDouble() - Math.PI)
                        .build()
        );
        sched.run();

        while (intake.pixelCount() > 0) {
            sched.addAction(new SequentialAction(
                    outtake.clawOpen(),
                    outtake.wristVerticalFlip(),
                    new SleepAction(0.2),
                    outtake.armStored(),
                    outtake.retractOuttakeBlocking(),
                    outtake.clawClosed(),
                    new SleepAction(0.3),
                    outtake.extendOuttakePartnerBlocking(),
                    outtake.armScoring(),
                    outtake.wristVerticalFlip()
            ));
        }

        // Wait for partner to move out of the way
        sched.addAction(new ActionUtil.RunnableAction(() -> intake.sideDistance(false) < 30));
        sched.addAction(drive.actionBuilder(detectPartner)
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                .build());
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
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, off)), AutoConstants.blueScoring[SPIKE].heading, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(12, new SequentialAction(
                        outtake.clawHalfOpen()
                ))
                .build()
        );
        sched.run();
    }

    private void intakeStack() {
        // Drive to stack & intake
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .afterDisp(5, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        new SleepAction(0.3),
                        outtake.clawOpen(),
                        outtake.retractOuttakeBlocking(),
                        outtake.extendOuttakeBarelyOut()
                ))
                .afterDisp(10, outtake.retractOuttake())
                .splineToConstantHeading(pastTruss.position, pastTruss.heading, drive.slowestVelConstraint, drive.slowestAccelConstraint)
                .afterDisp(0, new SequentialAction(
                        intake.feedClosed()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .afterDisp(0, intake.prepIntakeCount(false, false))
                .splineToSplineHeading(stack, stack.heading, drive.slowVelConstraint)
                .build()
        );
        sched.run();


        sched.addAction(intake.intakeCount(true));
        sched.run();

        if (intake.pixelCount() == 0) {
            tryAgain(stack, false, false);
        }
    }

    private void tryAgain(Pose2d current, boolean start, boolean one) {
        sched.addAction(drive.actionBuilder(current)
                .afterDisp(0, intake.intakeReverse())
                .strafeToLinearHeading(current.position.plus(new Vector2d(6, 0)), current.heading)
                .afterDisp(0, intake.prepIntakeCount(start, one))
                .strafeToLinearHeading(current.position, current.heading)
                .build());
        sched.addAction(intake.intakeCount(true));
    }

    private void cycle() {
        int pixelCount = intake.pixelCount();

        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToSplineHeading(intermediate, intermediate.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(1, new SequentialAction(
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed()
                ))
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                .afterDisp(1, new SequentialAction(
                        intake.intakeOff()
                ))
                .splineToConstantHeading(detectPartner.position, detectPartner.heading.toDouble() - Math.PI)
                .afterDisp(0, pixelCount > 0 ? new SequentialAction(
                        outtake.extendOuttakeCycleBlocking(),
                        outtake.armScoring(),
                        intake.feedOpen()
                ) : new SequentialAction())
                .build()
        );
        sched.run();

        if (pixelCount == 0) {
            return;
        }

        // Wait for partner to move out of the way
        sched.addAction(new ActionUtil.RunnableAction(() -> intake.sideDistance(false) < 24));
        sched.addAction(drive.actionBuilder(detectPartner)
                .splineToConstantHeading(scoring.position.plus(new Vector2d(-5, 0)), scoring.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .splineToConstantHeading(scoring.position, scoring.heading.toDouble() - Math.PI, drive.slowestVelConstraint, drive.slowestAccelConstraint)
                .build());
        sched.run();

        // Score
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed());
        sched.addAction(new SleepAction(0.4));
        sched.run();

        // Park
        sched.addAction(drive.actionBuilder(scoring)
                .afterDisp(1, new SequentialAction(
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ))
                .strafeToLinearHeading(park.position, park.heading)
                .build());
        sched.run();
    }
}
