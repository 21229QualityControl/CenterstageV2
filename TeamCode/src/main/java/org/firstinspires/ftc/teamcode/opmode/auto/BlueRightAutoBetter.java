package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.util.AutoConstants.blueScoring;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.ActionUtil;

@Config
@Disabled
@Autonomous(name = "Blue Right Auto 2+1", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAutoBetter extends AutoBase {
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
    public static Pose2d intermediate = new Pose2d(-48, 61, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-61.5, 35.5, Math.toRadians(180));

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
        sched.addAction(new SleepAction(7));
        deliverSpike();
        intakeStack();
        driveToScoring();
        scorePixels();
        park();
    }

    private void deliverSpike() {
        // if it doesn't need to deliver to the truss, the robot can just strafe.
        if (SPIKE != 2) {
            sched.addAction(
                    drive.actionBuilder(getStartPose())
                            .strafeTo(spike[SPIKE].position)
                            .build()
            );
        } else {
            // Otherwise, it needs to drive forwards then turn so it doesn't hit anything.
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

    private void intakeStack() {
        sched.addAction(
                new SequentialAction(
                        // move the robot to the stack and start the intake
                        drive.actionBuilder(spike[SPIKE])
                            .afterDisp(10, intake.intakeOn())
                            .afterDisp(10, outtake.clawOpen())
                            .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                            .strafeToLinearHeading(stack.position, stack.heading)
                            .build(),
                        new SequentialAction(
                            intake.stackClosed(),
                            new SleepAction(0.3),
                            intake.stackOpen(),
                            new SleepAction(0.3)
                        )
                )
        );
    }
    private void driveToScoring() {
        sched.addAction(
                new SequentialAction(
                        drive.actionBuilder(stack)
                                .afterDisp(7, new SequentialAction(
                                        intake.intakeOff(),
                                        outtake.clawClosed()
                                ))
                                .strafeToLinearHeading(intermediate.position, intermediate.heading)
                                .strafeToLinearHeading(new Vector2d(
                                        (blueScoring[SPIKE].position.x*2 + intermediate.position.x)/3,
                                        intermediate.position.y), blueScoring[SPIKE].heading)
                                .strafeToLinearHeading(blueScoring[(SPIKE + 1) % 3].position,
                                        blueScoring[(SPIKE + 1) % 3].heading)
                                .afterDisp(1, new ActionUtil.RunnableAction(() -> {
                                    double dist = frontSensors.backdropDistance();
                                    if (dist > 15) {
                                        dist = 9; // failed
                                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                                    }
                                    Log.d("BACKDROP_DIST", String.valueOf(dist));
                                    drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(11.25 - dist, 0)), drive.pose.heading);
                                    drive.updatePoseEstimate();
                                    return false;
                                }))
                                .build()
                )
        );
    }

    private void scorePixels() {
        sched.addAction(
                new SequentialAction(
                        outtake.wristScoring(),
                        outtake.extendOuttakeLowBlocking(),
                        drive.actionBuilder(blueScoring[(SPIKE + 1) % 3])
                                .strafeToLinearHeading(blueScoring[(SPIKE + 1) % 3].position.plus(new Vector2d(10, 0)),
                                        blueScoring[(SPIKE + 1) % 3].heading) // Correct for any turning that occured during the previous move
                                .build(),
                        // score the white pixel
                        outtake.clawOpen(),
                        new SleepAction(0.4),
                        outtake.extendOuttakeMidLow(),
                        new SleepAction(0.7),
                        // move robot out of backdrop so it can strafe
                        drive.actionBuilder(blueScoring[(SPIKE + 1) % 3])
                                .strafeToLinearHeading(blueScoring[SPIKE].position, blueScoring[SPIKE].heading)
                                .strafeToLinearHeading(blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), blueScoring[SPIKE].heading) // Strafe so the white doesn't block the yellow pixel.
                                .build(),
                        new SleepAction(0.5),
                        outtake.clawOpen(),
                        new SleepAction(0.2),
                        outtake.extendOuttakeMidBlocking()
                )
        );
    }

    private void park() {
        sched.addAction(
                drive.actionBuilder(new Pose2d(blueScoring[SPIKE].position.plus(new Vector2d(9, -3)),
                                blueScoring[SPIKE].heading))
                        .strafeToLinearHeading(blueScoring[SPIKE].position, blueScoring[SPIKE].heading)
                        .afterDisp(10, new SequentialAction(
                                outtake.wristStored(),
                                new SleepAction(0.5),
                                outtake.retractOuttake(),
                                outtake.clawClosed(),
                                new SleepAction(0.5)
                        ))
//                        .strafeToLinearHeading(new Vector2d(blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
//                        .strafeToLinearHeading(parking.position, parking.heading)
                        .build()
        );
    }
}
