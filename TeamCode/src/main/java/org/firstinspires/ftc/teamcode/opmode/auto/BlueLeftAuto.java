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

@Autonomous(name = "Blue Left Auto", group = "Auto")
public class BlueLeftAuto extends AutoBase {
    public static Pose2d start = new Pose2d(12, 63, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(7, 29, Math.toRadians(180)),
            new Pose2d(26, 22, Math.toRadians(180)),
            new Pose2d(36, 26, Math.toRadians(180))
    };
    public static Pose2d intermediate = new Pose2d(24, 8, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(-36, 8, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-57, 8, Math.toRadians(180));
    public static Pose2d secondStack = new Pose2d(-60, 14, Math.toRadians(150));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Left Auto");
    }

    @Override
    protected void onRun() {
        SPIKE = 1;

        scorePreload();

        // First stack
        intakeStack(true, false);
        cycle(false);

        intakeStack(false, false);
        cycle(false);

        // Second stack
        intakeStack(true, true);
        cycle(true);

        /*intakeStack(false, true);
        cycle(true);*/
    }

    private void scorePreload() {
        // Deliver spike
        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());
        sched.addAction(outtake.clawSingleClosed());
        //sched.addAction(new SleepAction(0.3));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                        .build()
        );
        sched.addAction(intake.wristStored());
        //sched.addAction(new SleepAction(0.2));
        sched.run();

        // Score preload
        sched.addAction(drive.actionBuilder(spike[SPIKE])
                .afterDisp(0, new SequentialAction(
                        outtake.extendOuttakeCloseBlocking(),
                        outtake.armScoring(),
                        outtake.wristSideways(false)
                ))
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                        .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.setPixelCount(0));
        sched.run();
    }

    private void intakeStack(boolean first, boolean nextStack) {
        TrajectoryActionBuilder bld = drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading))
                .afterDisp(1, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .afterDisp(0, new SequentialAction(
                        intake.prepIntakeCount(first, false),
                        outtake.extendOuttakeBarelyOut()
                ))
                .splineToConstantHeading(pastTruss.position, pastTruss.heading);
        if (nextStack) {
            bld = bld.splineToSplineHeading(secondStack, secondStack.heading, drive.slowVelConstraint);
        } else {
            bld = bld.splineToConstantHeading(stack.position, stack.heading, drive.slowVelConstraint);
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
                intake.intakeCount(),
                outtake.retractOuttake()
        ));
        sched.run();
    }

    private void cycle(boolean second) {
        sched.addAction(intake.intakeOff());
        TrajectoryActionBuilder bld = drive.actionBuilder(second ? secondStack : stack).setReversed(true);
        if (second) {
            bld = bld.splineToSplineHeading(pastTruss, pastTruss.heading.toDouble() - Math.PI);
        } else {
            bld = bld.splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI);
        }
        sched.addAction(bld
                .afterDisp(0, outtake.clawClosed())
                .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeCycleBlocking(),
                                outtake.armScoring(),
                                outtake.wristSideways(false),
                                intake.feedOpen()
                        ))
                .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading.toDouble() - Math.PI)
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed()); // Open & close feed in case outtake doesn't grab
        sched.addAction(intake.setPixelCount(0));
        sched.run();
    }
}
