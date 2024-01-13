package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

import java.util.Vector;

@Config
@Autonomous(name = "Blue Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto extends AutoBase {
   public static Pose2d[] spike = {
           new Pose2d(4, 36, Math.toRadians(45)),
           new Pose2d(12, 33, Math.toRadians(90)),
           new Pose2d(24, 42, Math.toRadians(90))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {
           new Pose2d(17, 46, Math.toRadians(45)),
           new Pose2d(12, 44, Math.toRadians(90)),
           new Pose2d(24, 55, Math.toRadians(90))};

   public static Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
   public static Pose2d parking = new Pose2d(56, 60, Math.toRadians(180));
   public static Pose2d[] stackPositions = {
           new Pose2d(-50.5, 16, Math.toRadians(180)),
           new Pose2d(-50.5, 14, Math.toRadians(180)),
           new Pose2d(-50.5, 16, Math.toRadians(180))
   };
   public static Pose2d stack;

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
      stack = stackPositions[SPIKE];
      deliverSpike();
      scorePreload(false);
      intakeStack();
      scorePreload(true);
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
      sched.addAction(
              new SequentialAction(
                      new SleepAction(0.5),
                      drive.actionBuilder(spike[SPIKE])
                              .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                              .build()
              )
      );
   }

   private void scorePreload(boolean mid) {
      if (mid) {
         SPIKE = (SPIKE + 1) % 3;
      }
      sched.addAction(
              new SequentialAction(
                      outtake.wristScoring(),
                      mid ? outtake.extendOuttakeTeleopBlocking() : outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.latchOpen(),
                      new SleepAction(0.5),
                      outtake.extendOuttakeMidBlocking()
              )
      );
   }

   private void intakeStack() {
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                              .afterDisp(10, new SequentialAction(
                                      outtake.wristStored(),
                                      new SleepAction(0.5),
                                      outtake.retractOuttake(),
                                      outtake.latchOpen(),
                                      new SleepAction(0.5)
                              ))
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                              .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y), stack.heading)
                              .afterDisp(60, intake.intakeOn())
                              .strafeToLinearHeading(stack.position, stack.heading)
                              .strafeToLinearHeading(stack.position.plus(new Vector2d(-10, 0)), stack.heading)
                              .build(),
                      new SequentialAction(
                              new SleepAction(0.6),
                              intake.stackClosed(),
                              new SleepAction(0.6),
                              intake.stackOpen(),
                              new SleepAction(0.6),
                              intake.stackClosed(),
                              new SleepAction(1.0),
                              intake.stackOpen()
                      ),
                      drive.actionBuilder(stack)
                              .afterTime(2, new SequentialAction(
                                      intake.intakeOff(),
                                      outtake.latchClosed()
                              ))
                              .strafeToLinearHeading(new Vector2d((AutoConstants.blueScoring[SPIKE].position.x*4 + stack.position.x)/5, stack.position.y), AutoConstants.blueScoring[SPIKE].heading)
                              .strafeToLinearHeading(AutoConstants.blueScoring[(SPIKE + 1) % 3].position, AutoConstants.blueScoring[(SPIKE + 1) % 3].heading)
                              .build()
              )
      );
   }

   private void park() {
      sched.addAction(
                   drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)),
                                   AutoConstants.blueScoring[SPIKE].heading))
                           .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                           .afterDisp(10, new SequentialAction(
                                   outtake.wristStored(),
                                   new SleepAction(0.5),
                                   outtake.retractOuttake(),
                                   outtake.latchClosed(),
                                   new SleepAction(0.5)
                           ))
                           .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                           .strafeToLinearHeading(parking.position, parking.heading)
                           .build()
      );
   }
}
