package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Autonomous(name = "Blue Right Auto", group = "Auto")
public class BlueRightAuto extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-48, 40, Math.toRadians(-90)),
            new Pose2d(-36, 40, Math.toRadians(-90)),
            new Pose2d(-35, 40, Math.toRadians(0))};
    public static Pose2d intermediate = new Pose2d(-36, 60, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(24, 60, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-58, 34, Math.toRadians(180));
    public static Pose2d corner = new Pose2d(50, 60, Math.toRadians(180));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Right Auto");
    }

    @Override
    protected void onRun() {
        SPIKE = 1;

        firstCycle();

        intakeStack(true);
        cycle();

        intakeStack(false);
        cycle();
    }

    private void firstCycle() {
        // Deliver spike
        sched.addAction(intake.setPixelCount(1));
        sched.addAction(intake.wristPreload());
        sched.addAction(new SleepAction(0.5));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToLinearHeading(spike[SPIKE].position.plus(new Vector2d(0, -4)), spike[SPIKE].heading) // It doesn't go far enough? Perhaps roadrunner needs to be tuned better
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.5));
        sched.run();

        // Intake stack
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .afterDisp(0, outtake.extendOuttakeBarelyOut())
                        .splineToLinearHeading(new Pose2d(-36, 45, Math.toRadians(180)), intermediate.heading)
                        .afterDisp(0, intake.prepIntakeCount(true, true))
                        .splineToLinearHeading(stack, stack.heading, drive.slowVelConstraint)
                        .build()
        );
        sched.addAction(intake.intakeCount());
        sched.run();

        // Drive to preload
        sched.addAction(
                drive.actionBuilder(stack)
                        .setReversed(true)
                        .afterDisp(0, outtake.retractOuttakeBlocking())
                        .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                        .afterDisp(0, outtake.clawClosed())
                        .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                        .afterDisp(1, new SequentialAction(
                                intake.intakeOff(),
                                outtake.extendOuttakePartnerBlocking(),
                                outtake.armScoring()
                        ))
                        .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading.toDouble() - Math.PI)
                        .build()
        );
        sched.run();

        // Detect spike
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            this.preloadProcessor.updateTarget(SPIKE, false);
            this.preloadProcessor.detecting = false;
            // TODO: Use back camera
            this.preloadPortal.setProcessorEnabled(this.aprilTagProcessor, true);
            this.preloadPortal.setProcessorEnabled(this.preloadProcessor, true);
            this.preloadPortal.resumeStreaming();
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            return false;
        }));
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            /*if (preloadProcessor.detecting) {
                Log.d("BACKDROP_PRELOADLEFT", String.valueOf(preloadProcessor.preloadLeft));
                this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                this.portal.stopStreaming();
                return false;
            }
            return true;*/
            return false;
        }));
        sched.addAction(outtake.wristSideways(!preloadProcessor.preloadLeft));
        sched.run();

        // Score
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                .afterDisp(12, new SequentialAction(
                        outtake.clawOpen(),
                        intake.setPixelCount(0)
                ))
                .build()
        );
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        sched.addAction(drive.actionBuilder(first ? AutoConstants.blueScoring[SPIKE] : corner)
                .afterDisp(0, first ? new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        new SleepAction(0.3),
                        outtake.retractOuttakeBlocking(),
                        outtake.extendOuttakeBarelyOut()
                ) : new SequentialAction())
                .afterDisp(10, outtake.retractOuttake())
                .splineToConstantHeading(pastTruss.position, pastTruss.heading, first ? drive.slowVelConstraint : drive.defaultVelConstraint, first ? drive.slowAccelConstraint : drive.defaultAccelConstraint)
                .afterDisp(0, intake.feedClosed())
                .afterDisp(0, intake.setPixelCount(0))
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .afterDisp(0, intake.prepIntakeCount(false, false))
                .splineToConstantHeading(stack.position, stack.heading, drive.slowVelConstraint)
                .build()
        );
        sched.addAction(intake.intakeCount());
        sched.run();

        // Update for future cycles
        if (first) {
            SPIKE = (SPIKE + 1) % 3;
        }
    }

    private void cycle() {
        sched.addAction(intake.intakeOff());
        // Drive to corner & open feed
        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
//                .afterDisp(0, outtake.extendOuttakeCloseBlocking())
                .afterDisp(10, new SequentialAction(
                        intake.feedOpen(),
                        intake.setPixelCount(0)
                ))
                .splineToConstantHeading(corner.position, corner.heading.toDouble() - Math.PI)
                .build()
        );
        sched.run();
    }
}
