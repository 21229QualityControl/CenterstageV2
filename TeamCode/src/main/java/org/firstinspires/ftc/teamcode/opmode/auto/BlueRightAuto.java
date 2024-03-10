package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

public class BlueRightAuto extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-48, 40, Math.toRadians(-90)),
            new Pose2d(-36, 30, Math.toRadians(-90)),
            new Pose2d(-31, 30, Math.toRadians(0))};
    public static Pose2d intermediate = new Pose2d(-36, 48, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(24, 48, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-57, 36, Math.toRadians(180));
    public static Pose2d corner = new Pose2d(57, 48, Math.toRadians(180));

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
        firstCycle();

        intakeStack(true);
        cycle();

        intakeStack(false);
        cycle();
    }

    private void firstCycle() {
        // Deliver spike
        sched.addAction(intake.wristDown());
        sched.addAction(new SleepAction(0.5));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeTo(spike[SPIKE].position)
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.5));
        sched.run();

        // Intake stack
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .splineToSplineHeading(intermediate, intermediate.heading)
                        .afterDisp(0, intake.intakeOn())
                        .splineToConstantHeading(stack.position, stack.heading)
                        .build()
        );
        sched.addAction(intake.intakeCount(true));
        sched.run();

        // Drive to preload
        sched.addAction(
                drive.actionBuilder(stack)
                        .splineToConstantHeading(intermediate.position, intermediate.heading)
                        .afterDisp(0, outtake.clawSingleClosed())
                        .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakePartnerBlocking(),
                                outtake.armScoring()
                        ))
                        .splineTo(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .build()
        );
        sched.run();

        // Detect spike
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            this.preloadProcessor.updateTarget(SPIKE, false);
            this.preloadProcessor.detecting = false;
            this.portal.setProcessorEnabled(this.aprilTagProcessor, true);
            this.portal.setProcessorEnabled(this.preloadProcessor, true);
            this.portal.resumeStreaming();
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            return false;
        }));
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            if (preloadProcessor.detecting) {
                Log.d("BACKDROP_PRELOADLEFT", String.valueOf(preloadProcessor.preloadLeft));
                this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                this.portal.stopStreaming();
                return false;
            }
            return true;
        }));
        sched.addAction(outtake.wristSideways(!preloadProcessor.preloadLeft));
        sched.run();

        // Score
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                .build()
        );
        sched.addAction(new SequentialAction(
                outtake.clawOpen(),
                new SleepAction(0.3)
        ));
        sched.addAction(drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading))
                        .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .build()
        );
        sched.run();

        // Prepare for next cycles
        sched.addAction(new SequentialAction(
                outtake.wristVertical(),
                outtake.armStored(),
                new SleepAction(0.3),
                outtake.retractOuttakeBlocking()
        ));
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        sched.addAction(drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                .afterDisp(0, intake.feedClosed())
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .afterDisp(0, intake.intakeOn())
                .splineToConstantHeading(stack.position, stack.heading)
                .build()
        );
        sched.addAction(intake.intakeCount(false));
        sched.run();

        // Update for future cycles
        if (first) {
            SPIKE = (SPIKE + 1) % 3;
        }
    }

    private void cycle() {
        // Drive to corner & open feed
        sched.addAction(drive.actionBuilder(stack)
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                .splineToConstantHeading(corner.position, corner.heading)
                .build()
        );
        sched.addAction(new SequentialAction(
                intake.feedOpen(),
                new SleepAction(0.5)
        ));
        sched.run();
    }
}
