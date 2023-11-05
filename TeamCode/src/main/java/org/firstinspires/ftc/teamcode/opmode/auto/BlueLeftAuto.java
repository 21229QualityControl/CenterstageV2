package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Blue Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAuto extends AutoBase {
   public static Pose2d[] spike = {new Pose2d(9, 42, Math.toRadians(45)), new Pose2d(12, 41, Math.toRadians(90)), new Pose2d(24, 42, Math.toRadians(90))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {new Pose2d(17, 50, Math.toRadians(45)), new Pose2d(12, 49, Math.toRadians(90)), new Pose2d(24, 50, Math.toRadians(90))};
   public static Pose2d start = new Pose2d(12, 64, Math.toRadians(90));
   public static Pose2d[] scoring = {new Pose2d(42, 28, Math.toRadians(180)), new Pose2d(42, 36, Math.toRadians(180)), new Pose2d(42, 40, Math.toRadians(180))};
   public static Pose2d parking = new Pose2d(56, 58, Math.toRadians(180));

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
             new SleepAction(0.5)
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(spike[SPIKE])
                              .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                              .strafeToLinearHeading(scoring[SPIKE].position, scoring[SPIKE].heading)
                              .build(),
                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(scoring[SPIKE])
                              .strafeToLinearHeading(scoring[SPIKE].position.plus(new Vector2d(10, 0)), scoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.extendOuttakeMidBlocking()
              )
      );
   }

   private void park() {
      sched.addAction(
                   drive.actionBuilder(new Pose2d(scoring[SPIKE].position.plus(new Vector2d(10, 0)),
                                   scoring[SPIKE].heading))
                           .strafeToLinearHeading(scoring[SPIKE].position, scoring[SPIKE].heading)
                           .afterDisp(10, new SequentialAction(
                                   outtake.wristStored(),
                                   new SleepAction(0.5),
                                   outtake.retractOuttake(),
                                   outtake.latchClosed(),
                                   new SleepAction(0.5)
                           ))
                           .strafeToLinearHeading(new Vector2d(scoring[SPIKE].position.x, parking.position.y), parking.heading)
                           .strafeToLinearHeading(parking.position, parking.heading)
                           .build()
      );
   }
}
