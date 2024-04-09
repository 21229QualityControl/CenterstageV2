package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Autonomous(name = "Blue Left Auto 2+4", group = "Auto")
public class BlueLeftAuto2_4 extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-46, 44, Math.toRadians(-90)),
            new Pose2d(-36, 40, Math.toRadians(-90)),
            new Pose2d(-33, 41, Math.toRadians(-45))};
    public static Pose2d intermediate = new Pose2d(-36, 60, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(40, 60, Math.toRadians(180));
    public static Pose2d stackAfterPreload = new Pose2d(-59, 44, Math.toRadians(210));
    public static Pose2d stack = new Pose2d(-59, 39, Math.toRadians(205));
    public static Pose2d park = new Pose2d(50, 60, Math.toRadians(180));
    public static Pose2d scoring = new Pose2d(56, 45, Math.toRadians(160));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Left Auto 2+4");
    }

    @Override
    protected void onRun() {
        scorePreload();

        intakeStack(true);
        cycle();

        intakeStack(false);
        cycle();
    }

    private void scorePreload() {
        // Deliver spike
        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToSplineHeading(spike[SPIKE].position, spike[SPIKE].heading)
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.run();

        // Score preload
        sched.addAction(drive.actionBuilder(spike[SPIKE])
                .afterDisp(0, new SequentialAction(
                        outtake.extendOuttakeCloseBlocking(),
                        outtake.armScoring(),
                        outtake.wristSideways(SPIKE == 2)
                ))
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        sched.addAction(drive.actionBuilder(first ? AutoConstants.blueScoring[SPIKE] : scoring)
                .afterDisp(5, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        new SleepAction(0.3),
                        outtake.clawOpen(),
                        outtake.retractOuttakeBlocking(),
                        outtake.extendOuttakeBarelyOut()
                ))
                .afterDisp(10, outtake.retractOuttake())
                .splineToSplineHeading(pastTruss, pastTruss.heading, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(0, new SequentialAction(
                        intake.feedClosed()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .afterDisp(0, intake.prepIntakeCount(false, false))
                .splineToSplineHeading(stack, stack.heading, drive.slowVelConstraint)
                .build()
        );
        sched.run();


        sched.addAction(intake.intakeCount(false));
        sched.run();
    }

    private void cycle() {
        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToSplineHeading(intermediate, intermediate.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(1, new SequentialAction(
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed()
                ))
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                .afterDisp(1, new SequentialAction(
                        intake.intakeOff(),
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
}
