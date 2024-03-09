package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;

public class BlueRightAuto extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-48, 40, Math.toRadians(-90)),
            new Pose2d(-36, 33, Math.toRadians(-90)),
            new Pose2d(-31, 37, Math.toRadians(0))};

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
        deliverSpike();
    }

    private void deliverSpike() {
        sched.addAction(intake.wristDown());
        sched.addAction(new SleepAction(0.5));
        sched.addAction(
                drive.actionBuilder(getStartPose())
                        .strafeTo(spike[SPIKE].position)
                        .build()
        );
        sched.addAction(intake.dropPreload());
        sched.run();
    }
}
