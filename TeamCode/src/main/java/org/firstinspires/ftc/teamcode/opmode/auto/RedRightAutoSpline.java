package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Config
@Autonomous(name = "Red Right Auto 2+4", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedRightAutoSpline extends AutoBase {
    public static Pose2d[] spike = {new Pose2d(24, -36, Math.toRadians(-90)), new Pose2d(10, -28, Math.toRadians(-90)), new Pose2d(4, -32, Math.toRadians(-45))};
    public static Pose2d[] spikeBackedOut =  {new Pose2d(24, -46, Math.toRadians(-90)), new Pose2d(10, -38, Math.toRadians(-90)), new Pose2d(16, -42, Math.toRadians(-45))};
    // 0 = right, 1 = middle, 2 = left
    public static Pose2d start = new Pose2d(12, -61, Math.toRadians(-90));
    public static Pose2d parking = new Pose2d(
            60, -60, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-60, -11, Math.toRadians(180));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Red Right Auto");
    }

    @Override
    protected void onRun() {
        deliverSpike();
        scorePreload();
        intakeStack(true);
        scoreStack();
        intakeStack(false);
        scoreStack();
        park();
    }

    private void deliverSpike() {
        if (SPIKE != 0) {
            sched.addAction(
                    drive.actionBuilder(getStartPose())
                            .strafeTo(spike[SPIKE].position)
                            .build()
            );
        } else {
            sched.addAction(
                    drive.actionBuilder(getStartPose())
                            .lineToY((spike[SPIKE].position.y + getStartPose().position.y)/2)
                            .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                            .build()
            );
        }
    }

    private void scorePreload() {
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .splineToConstantHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                        .splineToSplineHeading(AutoConstants.redScoring[SPIKE], AutoConstants.redScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.wristScoring(),
                                outtake.extendOuttakeLowBlocking()
                        ))
                        .splineToConstantHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.redScoring[SPIKE].heading)
                        .build()
        );
        sched.addAction(
                new SequentialAction(
                        outtake.latchOpen(),
                        new SleepAction(0.3),
                        outtake.extendOuttakeMid(),
                        new SleepAction(0.3)
                )
        );
    }

    private void intakeStack(boolean firstCycle) {
        sched.addAction(
                drive.actionBuilder(new Pose2d(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.redScoring[SPIKE].heading))
                        .splineToConstantHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeMidBlocking(),
                                outtake.wristStored(),
                                new SleepAction(0.3),
                                outtake.retractOuttake(),
                                outtake.latchOpen(),
                                new SleepAction(0.3)
                        ))
                        .splineToConstantHeading(new Vector2d(AutoConstants.redScoring[SPIKE].position.x, stack.position.y), stack.heading)
                        .afterDisp(0, new SequentialAction(
                                intake.intakeOn(),
                                intake.stackHalf()
                        ))
                        .splineToConstantHeading(stack.position, stack.heading)
                        .afterDisp(0, new SequentialAction(
                                intake.stackClosed(),
                                new SleepAction(0.3),
                                intake.stackHalf(),
                                new SleepAction(0.3),
                                intake.stackClosed(),
                                new SleepAction(0.3),
                                intake.stackHalf()
                        ))
                        .build()
        );
        if (firstCycle) {
            SPIKE = (SPIKE + 1) % 3;
        }
    }

    private void scoreStack() {
        sched.addAction(
                drive.actionBuilder(stack)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(AutoConstants.redScoring[SPIKE].position.x - 12, stack.position.y), stack.heading)
                        .afterDisp(0, new SequentialAction(
                                intake.intakeOff(),
                                outtake.latchClosed(),
                                intake.stackOpen()
                        ))
                        .splineToConstantHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeTeleopBlocking(),
                                new SleepAction(0.1),
                                outtake.wristScoring()
                        ))
                        .splineToConstantHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.redScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.latchOpen(),
                                outtake.extendOuttakeMid(),
                                new SleepAction(0.3)
                        ))
                        .build()
        );
    }

    private void park() {
        sched.addAction(
                drive.actionBuilder(new Pose2d(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.redScoring[SPIKE].heading))
                        .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(5, 0)), AutoConstants.redScoring[SPIKE].heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.wristStored(),
                                outtake.latchClosed(),
                                outtake.retractOuttake()
                        ))
                        .build()
        );
    }
}
