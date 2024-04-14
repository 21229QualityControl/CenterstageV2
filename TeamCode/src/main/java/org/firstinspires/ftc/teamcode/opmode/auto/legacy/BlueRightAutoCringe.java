package org.firstinspires.ftc.teamcode.opmode.auto.legacy;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous(name = "Blue Right Auto Cringe", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAutoCringe extends AutoBase {
    public static Pose2d start = new Pose2d(-36, 64, Math.toRadians(-90));
    public static Pose2d[] spike = {
            new Pose2d(-48, 38, Math.toRadians(-90)),
            new Pose2d(-36, 38, Math.toRadians(-90)),
            new Pose2d(-31, 48, Math.toRadians(0))};
    // 0 = right, 1 = middle, 2 = left
    public static Pose2d[] spikeBackedOut =  {
            new Pose2d(-48, 50, Math.toRadians(90)),
            new Pose2d(-36, 53, Math.toRadians(90)),
            new Pose2d(-40, 47, Math.toRadians(135))};
    public static Pose2d parking = new Pose2d(60, 12, Math.toRadians(180)); // Center
    //public static Pose2d parking = new Pose2d(60, 62, Math.toRadians(180)); // Corner
    public static Pose2d intermediate = new Pose2d(-40, 61, Math.toRadians(180));

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "Blue Right Auto 2+0");
    }

    @Override
    protected void onRun() {
        //sched.addAction(new SleepAction(10));
        deliverSpike();
        driveToScoring();
        scorePreload();
        park();
    }

    private void deliverSpike() {
        sched.addAction(intake.wristDown());
        sched.addAction(new SleepAction(0.5));
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
        sched.addAction(intake.wristStored());
        sched.addAction(new SleepAction(0.5));
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
                        new ActionUtil.RunnableAction(() -> {
                            this.preloadProcessor.updateTarget(SPIKE, false);
                            this.preloadProcessor.detecting = false;
                            this.portal.setProcessorEnabled(this.aprilTagProcessor, true);
                            this.portal.setProcessorEnabled(this.preloadProcessor, true);
                            this.portal.resumeStreaming();
                            return false;
                        }),
                        outtake.extendOuttakeBarelyOut(),
                        new SleepAction(0.5),
                        outtake.clawSingleClosed(),
                        new SleepAction(0.6)
                )
        );
    }

    private void scorePreload() {
        sched.addAction(
                new SequentialAction(
                        outtake.armScoring(),
                        outtake.extendOuttakeTeleopBlocking()
        ));
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
        sched.addAction(new SequentialAction(
                    new SleepAction(0.1),
                    drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                            .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)),
                                    AutoConstants.blueScoring[SPIKE].heading) // Correct for any turning that occurred during the previous move
                            .build(),
                    outtake.clawOpen(),
                    new SleepAction(0.5)
                )
        );
    }

    private void park() {
        sched.addAction(
                drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(12, 0)),
                                AutoConstants.blueScoring[SPIKE].heading))
                        .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                        .afterDisp(10, new SequentialAction(
                                outtake.armStored(),
                                outtake.wristVertical(),
                                new SleepAction(0.5),
                                outtake.retractOuttake(),
                                new SleepAction(0.5)
                        ))
                        .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                        .strafeToLinearHeading(parking.position, parking.heading)
                        .build()
        );
    }
}