package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedRightAuto extends AutoBase {
   public static Vector2d[] spike = {new Vector2d(12, -24), new Vector2d(12, -32), new Vector2d(12, -40)};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d start = new Pose2d(12, -64, Math.toRadians(-90));
   public static Pose2d[] scoring = {new Pose2d(40, -36, Math.toRadians(180)), new Pose2d(40, -36, Math.toRadians(180)), new Pose2d(40, -36, Math.toRadians(180))};
   public static Pose2d parking = new Pose2d(64, -60, Math.toRadians(180));

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
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(getStartPose())
                              .strafeTo(spike[SPIKE])
                              .build(),
                      intake.autoLatchOpen()
              )
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      drive.actionBuilder(new Pose2d(spike[SPIKE], getStartPose().heading))
                              .lineToY(spike[SPIKE].y - 4)
                              .strafeToLinearHeading(scoring[SPIKE].position, scoring[SPIKE].heading)
                              .build(),
                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(scoring[SPIKE])
                              .lineToX(scoring[SPIKE].position.x + 8)
                              .build(),
                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.wristStored(),
                      new SleepAction(0.5),
                      outtake.retractOuttake(),
                      outtake.latchClosed(),
                      new SleepAction(0.5)
              )
      );
   }

   private void park() {
      sched.addAction(
              drive.actionBuilder(new Pose2d(scoring[SPIKE].position.plus(new Vector2d(8, 0)),
                  scoring[SPIKE].heading))
                      .strafeTo(scoring[SPIKE].position)
                      .strafeTo(parking.position)
                      .build()
      );
   }
}
