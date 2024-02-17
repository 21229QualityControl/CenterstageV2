package org.firstinspires.ftc.teamcode.opmode.auto.legacy;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Config
//@Disabled
@Autonomous(name = "Blue Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAuto extends AutoBase {
    public static Pose2d[] spike = {
            new Pose2d(-48, 40, Math.toRadians(90)),
            new Pose2d(-36, 33, Math.toRadians(90)),
            new Pose2d(-31, 37, Math.toRadians(135))};
    // 0 = right, 1 = middle, 2 = left
    public static Pose2d[] spikeBackedOut =  {
            new Pose2d(-48, 50, Math.toRadians(90)),
            new Pose2d(-36, 53, Math.toRadians(90)),
            new Pose2d(-40, 47, Math.toRadians(135))};
    public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(90));
    public static Pose2d parking = new Pose2d(60, 12, Math.toRadians(180)); // Center
    //public static Pose2d parking = new Pose2d(60, 62, Math.toRadians(180)); // Corner
    public static Pose2d intermediate = new Pose2d(-40, 61, Math.toRadians(180));

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
        sched.addAction(new SleepAction(10));
        deliverSpike();
        driveToScoring();
        scorePreload();
        park();
    }

    private void deliverSpike() {
        if (SPIKE != 2) {
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
        sched.addAction(
                new SleepAction(0.5)
        );
    }

    private void driveToScoring() {
        sched.addAction(
                drive.actionBuilder(spike[SPIKE])
                        .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                        .strafeToLinearHeading(intermediate.position, intermediate.heading)
                        .strafeToLinearHeading(new Vector2d((AutoConstants.blueScoring[SPIKE].position.x*2 + intermediate.position.x)/3, intermediate.position.y), AutoConstants.blueScoring[SPIKE].heading)
                        .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .build()
        );
        sched.addAction(
                new SequentialAction(
                        outtake.clawPreloadlosed(),
                        new SleepAction(0.6)
                )
        );
    }

    private void scorePreload() {
        sched.addAction(
                new SequentialAction(
                        outtake.wristScoring(),
                        outtake.extendOuttakeTeleopBlocking(),
                        new ActionUtil.RunnableAction(() -> {
                            this.portal.setProcessorEnabled(this.aprilTagProcessor, true);
                            this.portal.setProcessorEnabled(this.preloadProcessor, true);
                            this.preloadProcessor.updateTarget(SPIKE, false);
                            return false;
                        }),
                        new SleepAction(1),
                        new ActionUtil.RunnableAction(() -> {
                            outtake.clawSidewaysInstant(!preloadProcessor.preloadLeft);
                            Log.d("BACKDROP_PRELOADLEFT", String.valueOf(preloadProcessor.preloadLeft));
                            return false;
                        }),
                        new SleepAction(0.1),
                        drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                                .afterDisp(0, new ActionUtil.RunnableAction(() -> {
                                    double dist = frontSensors.backdropDistance();
                                    if (dist > 20) {
                                        dist = 16; // failed
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                                    }
                                    Log.d("BACKDROP_DIST", String.valueOf(dist));
                                    drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(14 - dist, 0)), drive.pose.heading);
                                    drive.updatePoseEstimate();
                                    return false;
                                }))
                                .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)), AutoConstants.blueScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                                .build(),
                        outtake.clawOpen(),
                        new SleepAction(0.5),
                        outtake.extendOuttakeMidBlocking()
                )
        );
    }

    private void park() {
        sched.addAction(
                drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)),
                                AutoConstants.blueScoring[SPIKE].heading))
                        .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .afterDisp(10, new SequentialAction(
                                outtake.wristStored(),
                                new SleepAction(0.5),
                                outtake.retractOuttake(),
                                outtake.clawClosed(),
                                new SleepAction(0.5)
                        ))
                        .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                        .strafeToLinearHeading(parking.position, parking.heading)
                        .build()
        );
    }
}
