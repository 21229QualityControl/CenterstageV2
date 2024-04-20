package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Autonomous(name = "Red Right Auto 2+6", group = "Auto")
public class RedRightAuto2_6 extends AutoBase {
    public static Pose2d start = new Pose2d(12, -63, Math.toRadians(90));
    public static Pose2d[] spike = {
            new Pose2d(34, -33, Math.toRadians(-180)),
            new Pose2d(28, -22, Math.toRadians(-180)),
            new Pose2d(9, -34, Math.toRadians(135))
    };
    public static Pose2d intermediate = new Pose2d(24,  -9, Math.toRadians(-180));
    public static Pose2d pastTruss = new Pose2d(-36, -9, Math.toRadians(-180));
    public static Pose2d stack = new Pose2d(-57, -13, Math.toRadians(-180));
    public static Pose2d secondStack = new Pose2d(-58, -20, Math.toRadians(-150));
    public static Pose2d scoring = new Pose2d(55, -27, Math.toRadians(-200));
    public static Pose2d park = new Pose2d(48, -26, Math.toRadians(-200));

    @Override
    protected Pose2d getStartPose() {
        return start;
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

        // Second stack (need 9 seconds)
        if (getRuntime() < 21) {
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
                .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(12, 0.3)), AutoConstants.redScoring[SPIKE].heading)
                        .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.run();
    }

    private void intakeStack(boolean first, boolean nextStack) {
        TrajectoryActionBuilder bld = drive.actionBuilder(first ? new Pose2d(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(12, 0.3)), AutoConstants.redScoring[SPIKE].heading) : scoring)
                .afterDisp(1, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ));
        if (first) {
            bld = bld.splineToConstantHeading(intermediate.position.plus(new Vector2d(0, -3)), intermediate.heading);
        } else {
            bld = bld.splineToSplineHeading(intermediate, intermediate.heading);
        }
        bld = bld.afterDisp(0, new SequentialAction(
                    intake.prepIntakeCount(first || nextStack, false),
                    nextStack ? intake.intakeOn() : intake.intakeSlow(),
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
                intake.intakeCount(false),
                outtake.retractOuttake()
        ));
        sched.run();
    }

    private void cycle(boolean second) {
        TrajectoryActionBuilder bld = drive.actionBuilder(second ? secondStack : stack).setReversed(true);
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
                                second ? outtake.extendOuttakeCycleHighBlocking() : outtake.extendOuttakeCycleBlocking(),
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
