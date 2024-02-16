package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Config
@Autonomous(name = "Blue Left Auto 2+2 Partner", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto2_2 extends AutoBase {
    public static Pose2d[] spike = {
            new Pose2d(7, 36, Math.toRadians(45)),
            new Pose2d(16, 32, Math.toRadians(90)),
            new Pose2d(24, 42, Math.toRadians(90))
    };
    // 0 = right, 1 = middle, 2 = left
    public static Pose2d[] spikeBackedOut = {
            new Pose2d(20, 46, Math.toRadians(45)),
            new Pose2d(16, 43, Math.toRadians(90)),
            new Pose2d(24, 55, Math.toRadians(90))
    };

    public static Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
    public static Pose2d parking = new Pose2d(58, 58, Math.toRadians(180));
    public static Pose2d stack = new Pose2d(-50.5, 11, Math.toRadians(180));
    public static double[] stackOffset = {0, 3.75, 3.75};
    public static double[] backdropDistOffset = {9.3, 8.5, 9};
    private int startingSpike;

   @Override
   protected Pose2d getStartPose() {
      return start;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Blue Left Auto 2+2 Partner");
   }

   @Override
   protected void onRun() {
       startingSpike = SPIKE;
       led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
       deliverSpike();
       scorePreload();
       sched.addAction(led.setPatternAction(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE));
       intakeStack(true);
       sched.addAction(new SleepAction(8));
       scoreStack(true);
       parkBackdrop();
       //park();
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
                         .splineToSplineHeading(spikeBackedOut[SPIKE], spikeBackedOut[SPIKE].heading)
                         .splineToLinearHeading(AutoConstants.blueScoring[SPIKE], AutoConstants.blueScoring[SPIKE].heading)
                         .afterDisp(0, new SequentialAction(
                                 outtake.wristScoring(),
                                 outtake.extendOuttakeLowBlocking()
                         ))
                         .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading)
                         .afterDisp(1, new ActionUtil.RunnableAction(() -> {
                             double dist = frontSensors.backdropDistance();
                             if (dist > 15) {
                                 dist = 9; // failed
                                 led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                             }
                             Log.d("BACKDROP_DIST", String.valueOf(dist));
                             offsetPos = new Vector2d(-1 * (backdropDistOffset[SPIKE] - dist), 0);
                             drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(backdropDistOffset[SPIKE] - dist, 0)), drive.pose.heading);
                             drive.updatePoseEstimate();
                             return false;
                         }))
                         .build()
         );
         sched.addAction(
                 new SequentialAction(
                         outtake.latchOpen(),
                         new SleepAction(0.3),
                         outtake.extendOuttakeMid(),
                         new SleepAction(0.1)
                 )
         );
   }

   private void intakeStack(boolean firstCycle) {
      sched.addAction(
              drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                      .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                      .afterDisp(0, new ActionUtil.RunnableAction(() -> {
                          drive.pose = new Pose2d(drive.pose.position.plus(offsetPos), drive.pose.heading); // Fix stack pos reliability
                          return false;
                      }))
                      .afterDisp(0, new SequentialAction(
                              outtake.extendOuttakeMidBlocking(),
                              outtake.wristStored(),
                              new SleepAction(0.3),
                              outtake.retractOuttake(),
                              outtake.latchOpen()
                      ))
                      .splineToConstantHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y), stack.heading)
                      .afterDisp(0, new SequentialAction(
                              intake.intakeOn()
                      ))
                      .splineToConstantHeading(stack.position, stack.heading, drive.speedVelConstraint, drive.speedAccelConstraint)
                      .strafeToLinearHeading(stack.position.plus(new Vector2d(-10, stackOffset[startingSpike])), stack.heading)
                      .afterDisp(8, new SequentialAction(
                              intake.stackClosed(),
                              new SleepAction(0.3),
                              intake.stackHalf(),
                              new SleepAction(0.2),
                              intake.stackClosed(),
                              new SleepAction(0.2),
                              intake.stackOpen(),
                              new SleepAction(0.3)
                      ))
                      .build()
      );
      if (firstCycle) {
         SPIKE = (SPIKE + 1) % 3;
      }
   }

   private Vector2d offsetPos;
   private void scoreStack(boolean firstCycle) {
      sched.addAction(
              drive.actionBuilder(stack)
                      .setReversed(true)
                      .splineToConstantHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x - 12, stack.position.y), stack.heading, drive.speedVelConstraint, drive.speedAccelConstraint)
                      .afterDisp(0, new SequentialAction(
                              intake.intakeOff(),
                              outtake.latchClosed()
                      ))
                      .afterDisp(10, new SequentialAction(
                              firstCycle ? outtake.extendOuttakeTeleopBlocking() : outtake.extendOuttakeMidBlocking(),
                              new SleepAction(0.1),
                              outtake.wristScoring()
                      ))
                      .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                      .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading)
                      .afterDisp(1, new ActionUtil.RunnableAction(() -> {
                          double dist = frontSensors.backdropDistance();
                          if (dist > 15) {
                              dist = 9; // failed
                          }
                          Log.d("BACKDROP_DIST", String.valueOf(dist));
                          offsetPos = new Vector2d(-1 * (10 - dist), 0);
                          drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(10 - dist, 0)), drive.pose.heading);
                          drive.updatePoseEstimate();
                          return false;
                      }))
                      .afterDisp(8, new SequentialAction(
                              outtake.latchOpen(),
                              new SleepAction(0.2),
                              outtake.latchClosed(),
                              new SleepAction(0.1),
                              outtake.extendOuttakeMid()
                      ))
                      .build()
      );
   }

   private void parkBackdrop() {
      sched.addAction(
           drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                   .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(5, 0)), AutoConstants.blueScoring[SPIKE].heading)
                   .afterDisp(0, new SequentialAction(
                           outtake.wristStored(),
                           outtake.latchClosed(),
                           outtake.retractOuttake()
                   ))
                   .build()
      );
   }

   private void park() {
       sched.addAction(
               drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                       .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                       .afterDisp(0, new SequentialAction(
                               outtake.wristStored(),
                               new SleepAction(0.2),
                               outtake.retractOuttake(),
                               outtake.latchClosed()
                       ))
                       .splineToConstantHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                       .splineToConstantHeading(parking.position, parking.heading)
                       .build()
       );
   }
}
