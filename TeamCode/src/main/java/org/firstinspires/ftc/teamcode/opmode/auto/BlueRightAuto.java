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
            new Pose2d(-48, 38, Math.toRadians(-90)),
            new Pose2d(-36, 38, Math.toRadians(-90)),
            new Pose2d(-31, 48, Math.toRadians(0))};
    public static Pose2d intermediate = new Pose2d(-36, 56, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(24, 56, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-58, 36, Math.toRadians(180));
    public static Pose2d corner = new Pose2d(50, 56, Math.toRadians(180));

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
        sched.addAction(intake.wristDown());
        sched.addAction(new SleepAction(0.5));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.5));
        sched.run();

        // Intake stack
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .splineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(180)), intermediate.heading)
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
                        .afterDisp(0, outtake.clawClosed())
                        .setTangent(pastTruss.heading)
                        .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                        .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakePartnerBlocking(),
                                outtake.armScoring(),
                                intake.setPixelCount(0)
                        ))
                        .build()
        );
        sched.run();

        // Detect spike
        sched.addAction(new ActionUtil.RunnableAction(() -> {
            this.preloadProcessor.updateTarget(SPIKE, false);
            this.preloadProcessor.detecting = false;
            // TODO: Use back camera
            this.portal.setProcessorEnabled(this.aprilTagProcessor, true);
            this.portal.setProcessorEnabled(this.preloadProcessor, true);
            this.portal.resumeStreaming();
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
                outtake.retractOuttake()
        ));
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        sched.addAction(drive.actionBuilder(first ? AutoConstants.blueScoring[SPIKE] : corner)
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
                .setReversed(true)
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                .splineToConstantHeading(corner.position, corner.heading)
                .build()
        );
        sched.addAction(new SequentialAction(
                intake.feedOpen(),
                intake.setPixelCount(0),
                new SleepAction(0.5)
        ));
        sched.run();
    }
}
