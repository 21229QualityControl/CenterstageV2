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
   public static Pose2d parking = new Pose2d(60, 62, Math.toRadians(180));
   public static Pose2d stack = new Pose2d(-50.25, 14, Math.toRadians(180));

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
      scorePreload(false);
      intakeStack();
      scorePreload(true);
      park();
   }

   private void deliverSpike() {
      // Push spike
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
      // Back away from spike
      sched.addAction(
              new SequentialAction(
                      new SleepAction(0.1), // do we even need this?
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
                      // release pixels one at a time to prevent them from bouncing and fall off the backdrop
                      outtake.latchScoring(),
                      new SleepAction(0.4),
                      outtake.latchOpen(),
                      new SleepAction(0.2),
                      outtake.extendOuttakeMidBlocking()
              )
      );
   }

   private void intakeStack() {
      sched.addAction(
              new SequentialAction(
                      // retract outtake and drive to stack
                      drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                              .afterDisp(10, new SequentialAction(
                                      outtake.wristStored(),
                                      new SleepAction(0.25),
                                      outtake.retractOuttake(),
                                      outtake.latchOpen()
                              ))
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                              .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y), stack.heading)
                              .afterDisp(60, intake.intakeOn())
                              .strafeToLinearHeading(stack.position, stack.heading)
                              .strafeToLinearHeading(stack.position.plus(new Vector2d(-10, 1.5)), stack.heading)
                              .build(),
                      // intake 2 pixels from the stack
                      new SequentialAction(
                              intake.stackClosed(), // first close
                              new SleepAction(0.6),
                              intake.stackHalf(), // open for second to fall
                              new SleepAction(0.4),
                              intake.stackClosed(), // second close
                              new SleepAction(0.6),
                              intake.stackOpen()
                      ),
                      // stop intake and drive to scoring position
                      drive.actionBuilder(stack)
                              .afterTime(2, new SequentialAction(
                                      intake.intakeOff(),
                                      outtake.latchClosed()
                              ))
                              .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y), AutoConstants.blueScoring[SPIKE].heading)
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
                              new SleepAction(0.2),
                              outtake.retractOuttake(),
                              outtake.latchClosed()
                      ))
                      .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                      .strafeToLinearHeading(parking.position, parking.heading)
                      .build()
      );
   }
}
