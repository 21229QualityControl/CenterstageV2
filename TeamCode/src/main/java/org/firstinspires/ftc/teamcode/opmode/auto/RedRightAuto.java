package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedRightAuto extends AutoBase {
   public static Vector2d[] spike = {new Vector2d(-40, -36), new Vector2d(-40, -36), new Vector2d(-40, -36)};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d start = new Pose2d(-64, -36, Math.toRadians(-90));
   public static Pose2d scoring = new Pose2d(-40, -56, Math.toRadians(-180));
   public static Pose2d parking = new Pose2d(-60, -64, Math.toRadians(-90));

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
      Actions.runBlocking(
              new SequentialAction(
                      drive.actionBuilder(getStartPose())
                              .lineToX(spike[SPIKE].x)
                              .build(),
                      intake.autoLatchOpen()
              )
      );
   }

   private void scorePreload() {
      Actions.runBlocking(
              new SequentialAction(
                      drive.actionBuilder(new Pose2d(spike[SPIKE], getStartPose().heading)).
                              strafeToLinearHeading(scoring.position, scoring.heading)
                              .build(),
                      outtake.wristScoring(),
                      outtake.extendOuttakeBlocking(),
                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.wristStored(),
                      outtake.retractOuttake(),
                      outtake.latchClosed()
              )
      );
   }

   private void park() {
      Actions.runBlocking(
              drive.actionBuilder(scoring)
                      .strafeTo(parking.position)
                      .build()
      );
   }
}
