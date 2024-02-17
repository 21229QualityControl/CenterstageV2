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
@Autonomous(name = "Blue Left Auto Cringe", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAutoCringe extends AutoBase {
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
   public static Pose2d parking = new Pose2d(56, 60, Math.toRadians(180));
   public static Pose2d stack = new Pose2d(-55, 16, Math.toRadians(180));

   @Override
   protected Pose2d getStartPose() {
      return start;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Blue Left Auto 2+0");
   }

   @Override
   protected void onRun() {
      deliverSpike();
      scorePreload(false);
      //intakeStack();
      //scorePreload(true);
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
      sched.addAction(
              new SequentialAction(
                      outtake.wristScoring(),
                      mid ? outtake.extendOuttakeTeleopBlocking() : outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                              .afterDisp(1, new ActionUtil.RunnableAction(() -> {
                                  double dist = frontSensors.backdropDistance();
                                  if (dist > 15) {
                                      dist = 12; // failed
                                      led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                                  }
                                  Log.d("BACKDROP_DIST", String.valueOf(dist));
                                  drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(11 - dist, 0)), drive.pose.heading);
                                  drive.updatePoseEstimate();
                                  return false;
                              }))
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.clawOpen(),
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
                                      outtake.clawOpen(),
                                      new SleepAction(0.5)
                              ))
                              .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y - 8), stack.heading)
                              .afterDisp(60, intake.intakeOn())
                              .strafeToLinearHeading(stack.position.plus(new Vector2d(30, -8)), stack.heading)
                              .strafeToLinearHeading(stack.position, stack.heading)
                              .build(),
                      new SleepAction(0.5),
                      intake.stackClosed(),
                      new SleepAction(0.3),
                      intake.stackOpen(),
                      new SleepAction(0.3),
                      intake.stackClosed(),
                      new SleepAction(0.3),
                      intake.stackOpen(),
                      drive.actionBuilder(stack)
                              .afterTime(2, new SequentialAction(
                                      intake.intakeOff(),
                                      outtake.clawClosed()
                              ))
                              .strafeToLinearHeading(new Vector2d((AutoConstants.blueScoring[SPIKE].position.x*4 + stack.position.x)/5, stack.position.y - 12), AutoConstants.blueScoring[SPIKE].heading)
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
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
                                   outtake.clawClosed(),
                                   new SleepAction(0.5)
                           ))
                           .strafeToLinearHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, parking.position.y), parking.heading)
                           .strafeToLinearHeading(parking.position, parking.heading)
                           .build()
      );
   }
}
