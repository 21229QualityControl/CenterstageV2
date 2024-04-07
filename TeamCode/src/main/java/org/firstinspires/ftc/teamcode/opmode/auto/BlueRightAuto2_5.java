package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Autonomous(name = "Blue Right Auto 2+5", group = "Auto")
public class BlueRightAuto2_5 extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-46, 44, Math.toRadians(-90)),
            new Pose2d(-36, 20, Math.toRadians(90)),
            new Pose2d(-33, 41, Math.toRadians(-45))};
    public static Pose2d intermediate = new Pose2d(24,  14, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(-36, 14, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-59, 12, Math.toRadians(180));
    public static Pose2d scoring = new Pose2d(56, 21, Math.toRadians(200));
    public static Pose2d park = new Pose2d(53, 22, Math.toRadians(180));

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
        cycle();

        intakeStack(false);
        cycle();

        park();
    }

    private void scorePreload() {
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
                        .setReversed(true)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeBarelyOut(),
                                intake.prepIntakeCount(true, true)
                        ))
                        .splineToLinearHeading(pastTruss, pastTruss.heading, drive.defaultVelConstraint, drive.slowAccelConstraint)
                        .splineToSplineHeading(stack, stack.heading)
                        .build()
        );
        sched.addAction(intake.intakeCount());
        sched.run();


        // Score preload
        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                .afterDisp(0, new SequentialAction(
                        outtake.clawClosed(),
                        intake.intakeOff()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading.toDouble() - Math.PI)
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
        TrajectoryActionBuilder bld = drive.actionBuilder(first ? new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading) : scoring)
                .afterDisp(1, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ));
        if (first) {
            bld = bld.splineToConstantHeading(intermediate.position, intermediate.heading);
        } else {
            bld = bld.splineToSplineHeading(intermediate, intermediate.heading);
        }
        bld = bld.afterDisp(0, new SequentialAction(
                    intake.prepIntakeCount(first, false),
                    intake.intakeOn(),
                    outtake.extendOuttakeBarelyOut()
            ))
            .splineToConstantHeading(pastTruss.position, pastTruss.heading)
            .splineToConstantHeading(stack.position, stack.heading, drive.slowVelConstraint);
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

    private void cycle() {
        sched.addAction(drive.actionBuilder(stack)
                .setReversed(true)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading.toDouble() - Math.PI)
                .afterDisp(0, new SequentialAction(
                        outtake.clawClosed(),
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
