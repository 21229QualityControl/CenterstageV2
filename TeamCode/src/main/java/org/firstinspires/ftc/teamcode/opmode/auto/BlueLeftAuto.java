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
   public static Vector2d[] spike = {new Vector2d(0, 34), new Vector2d(12, 34), new Vector2d(24, 40)};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d start = new Pose2d(12, 64, Math.toRadians(90));
   public static Pose2d[] scoring = {new Pose2d(30, 28, Math.toRadians(180)), new Pose2d(40, 34, Math.toRadians(180)), new Pose2d(40, 38, Math.toRadians(180))};
   public static Pose2d parking = new Pose2d(64, 60, Math.toRadians(180));

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
                    .strafeTo(spike[SPIKE])
                    .build()
         );
      } else {
         sched.addAction(
                 drive.actionBuilder(getStartPose())
                         .strafeTo(new Vector2d(12, 40))
                         .strafeTo(spike[SPIKE])
                         .build()
         );
      }
      sched.addAction(
              new SequentialAction(
                      intake.autoLatchRelease(),
                     new SleepAction(0.5)
              )
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(new Pose2d(spike[SPIKE], getStartPose().heading))
                              .lineToY(spike[SPIKE].y + 8)
                              .afterDisp(10, intake.autoLatchStore())
                              .strafeToLinearHeading(scoring[SPIKE].position, scoring[SPIKE].heading)
                              .build(),
                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(scoring[SPIKE])
                              //.lineToX(scoring[SPIKE].position.x + 8)
                              .strafeToLinearHeading(scoring[SPIKE].position.plus(new Vector2d(SPIKE != 0 ? 10 : 20, 0)), scoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.latchScoring(),
                      new SleepAction(0.5)
              )
      );
   }

   private void park() {
      sched.addAction(
                   drive.actionBuilder(new Pose2d(scoring[SPIKE].position.plus(new Vector2d(SPIKE != 0 ? 10 : 20, 0)),
                                   scoring[SPIKE].heading))
                           .afterDisp(8, new SequentialAction(
                                   outtake.wristStored(),
                                   new SleepAction(0.5),
                                   outtake.retractOuttake(),
                                   outtake.latchClosed()
                           ))
                           .strafeTo(scoring[SPIKE].position)
                           .strafeTo(parking.position)
                           .build()
      );
   }
}