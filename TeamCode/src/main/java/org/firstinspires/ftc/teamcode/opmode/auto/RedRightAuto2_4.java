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

@Autonomous(name = "Red Right Auto 2+4", group = "Auto")
public class RedRightAuto2_4 extends AutoBase {
    public static Pose2d start = new Pose2d(12, -63, Math.toRadians(90));
    public static Pose2d[] spike = {
            new Pose2d(35, -31, Math.toRadians(-180)),
            new Pose2d(26, -22, Math.toRadians(-180)),
            new Pose2d(9, -34, Math.toRadians(135))
    };
    public static Pose2d intermediate = new Pose2d(-36, -60, Math.toRadians(-180));
    public static Pose2d pastTruss = new Pose2d(26, -60, Math.toRadians(-180));
    public static Pose2d stack = new Pose2d(-59, -43, Math.toRadians(-210));
    public static Pose2d park = new Pose2d(48, -43, Math.toRadians(-180));
    public static Pose2d scoring = new Pose2d(56, -45, Math.toRadians(-160));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Red Right Auto 2+4");
    }

    @Override
    protected void onInit() {
        sched.addAction(outtake.clawSingleClosed());
        sched.run();
    }

    @Override
    protected void onRun() {
        scorePreload();

        intakeStack(true);
        cycle();

        intakeStack(false);
        cycle();

        park();
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
                        outtake.wristVerticalFlip()
                ))
                .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(12, -0.3)), AutoConstants.redScoring[SPIKE].heading, drive.slowVelConstraint)
                .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.run();
    }

    private void intakeStack(boolean first) {
        // Drive to stack & intake
        TrajectoryActionBuilder bld = drive.actionBuilder(first ? AutoConstants.redScoring[SPIKE] : scoring)
                .afterDisp(5, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        new SleepAction(0.3),
                        outtake.clawOpen(),
                        outtake.retractOuttakeBlocking(),
                        outtake.extendOuttakeBarelyOut()
                ))
                .afterDisp(10, outtake.retractOuttake());
        if (first) {
            bld = bld.splineToConstantHeading(pastTruss.position.plus(new Vector2d(0, 2)), pastTruss.heading, drive.slowVelConstraint, drive.slowAccelConstraint);
        } else {
            bld = bld.splineToSplineHeading(pastTruss, pastTruss.heading, drive.slowVelConstraint, drive.slowAccelConstraint);
        }
        sched.addAction(bld
                .afterDisp(0, new SequentialAction(
                        intake.feedClosed()
                ))
                .splineToConstantHeading(intermediate.position.plus(new Vector2d(0, first ? 2 : 0)), intermediate.heading, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(0, intake.prepIntakeCount(first, false))
                .splineToSplineHeading(stack, stack.heading, drive.slowVelConstraint, drive.slowAccelConstraint)
                .build()
        );
        sched.run();


        sched.addAction(intake.intakeCount(true));
        sched.run();


        if (intake.pixelCount() == 0) {
            tryAgain(stack, first);
        } else {
            if (first) {
                sched.addAction(new SleepAction(4));
            }
        }
    }

    private void tryAgain(Pose2d current, boolean start) {
        sched.addAction(drive.actionBuilder(current)
                .afterDisp(0, intake.intakeReverse())
                .strafeToLinearHeading(current.position.plus(new Vector2d(6, 0)), current.heading)
                .afterDisp(0, intake.prepIntakeCount(start, false))
                .strafeToLinearHeading(current.position, current.heading)
                .build());
        sched.addAction(intake.intakeCount(true));
        sched.run();
    }

    private void cycle() {
        sched.addAction(drive.actionBuilder(stack)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(intermediate, intermediate.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .afterDisp(1, new SequentialAction(
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed()
                ))
                .afterDisp(43, new SequentialAction(
                        outtake.extendOuttakeCycleBlocking(),
                        outtake.armScoring(),
                        intake.feedOpen(),
                        intake.intakeOff()

                ))
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
                .splineToSplineHeading(scoring, scoring.heading.toDouble() - Math.PI, drive.slowVelConstraint, drive.slowAccelConstraint)
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

