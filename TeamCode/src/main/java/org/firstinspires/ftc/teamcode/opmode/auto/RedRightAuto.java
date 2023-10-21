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
   public static Pose2d pixel;
   public static Pose2d[] parking = {};
   public static Pose2d scoring = new Pose2d(-52, -24, Math.toRadians(-90));

   @Override
   protected Pose2d getStartPose() {
      return new Pose2d(-64, -36, Math.toRadians(-90));
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Red Right Auto");
   }

   @Override
   protected void onRun() {
      deliverSpike();
      scorePreload();
   }

   private void deliverSpike() {
      Actions.runBlocking(
              new SequentialAction(
                      drive.actionBuilder(getStartPose())
                              .splineTo(parking[SPIKE].position, parking[SPIKE].heading)
                              .build(),
                      intake.autoLatchOpen()
              )
      );
   }

   private void scorePreload() {
      Actions.runBlocking(
              new SequentialAction(
                      drive.actionBuilder(parking[SPIKE]).
                              splineTo(scoring.position, scoring.heading)
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
}
