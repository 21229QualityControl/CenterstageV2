package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

public class BlueLeftAuto extends AutoBase {
    public static Pose2d start = new Pose2d(12, 63, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(7, 36, Math.toRadians(180)),
            new Pose2d(16, 32, Math.toRadians(180)),
            new Pose2d(24, 42, Math.toRadians(180))
    };
    public static Pose2d intermediate = new Pose2d(36, 10, Math.toRadians(180));
    public static Pose2d pastTruss = new Pose2d(-36, 10, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-57, 12, Math.toRadians(180));
    public static Pose2d secondStack = new Pose2d(-50, 24, Math.toRadians(135));

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
        scorePreload();

        // First stack
        intakeStack(true, false);
        cycle();

        intakeStack(false, false);
        cycle();

        // Second stack
        intakeStack(true, true);
        cycle();

        intakeStack(false, true);
        cycle();
    }

    private void scorePreload() {
        // Deliver spike
        sched.addAction(intake.wristDown());
        sched.addAction(outtake.clawSingleClosed());
        sched.addAction(new SleepAction(0.5));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeToLinearHeading(spike[SPIKE].position, spike[SPIKE].heading)
                        .build()
        );
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.5));
        sched.run();

        // Score preload
        sched.addAction(outtake.extendOuttakeCloseBlocking());
        sched.addAction(outtake.armScoring());
        sched.addAction(outtake.wristSideways(false));
        sched.addAction(drive.actionBuilder(spike[SPIKE])
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                        .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.run();
    }

    private void intakeStack(boolean first, boolean nextStack) {
        sched.addAction(drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading))
                .afterDisp(1, new SequentialAction(
                        outtake.wristVertical(),
                        outtake.armStored(),
                        outtake.retractOuttakeBlocking()
                ))
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                .afterDisp(0, intake.intakeOn())
                .splineToConstantHeading((nextStack ? secondStack : stack).position, (nextStack ? secondStack : stack).heading)
                .build()
        );
        if (first) {
            SPIKE = (SPIKE + 1) % 3;
        }
        sched.addAction(intake.intakeCount(first));
        sched.run();
    }

    private void cycle() {
        sched.addAction(drive.actionBuilder(stack)
                .splineToConstantHeading(pastTruss.position, pastTruss.heading)
                .afterDisp(0, outtake.clawClosed())
                .splineToConstantHeading(intermediate.position, intermediate.heading)
                        .afterDisp(0, new SequentialAction(
                                outtake.extendOuttakeCycleBlocking(),
                                outtake.armScoring(),
                                outtake.wristSideways(false)
                        ))
                .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading)
                .build()
        );
        sched.addAction(outtake.clawOpen());
        sched.run();
    }
}
