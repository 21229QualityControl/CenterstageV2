package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AutoConstants;

import java.util.Vector;

@Config
@Autonomous(name = "Blue Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueRightAuto extends AutoBase {
   public static Pose2d[] spike = {new Pose2d(-24, 42, Math.toRadians(90)), new Pose2d(-12, 41, Math.toRadians(90)), new Pose2d(-9, 42, Math.toRadians(-45))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {new Pose2d(-24, 50, Math.toRadians(90)), new Pose2d(-12, 49, Math.toRadians(90)), new Pose2d(-17, 50, Math.toRadians(-45))};
   public static Pose2d start = new Pose2d(-12, 64, Math.toRadians(90));
   public static Pose2d parking = new Pose2d(56, 12, Math.toRadians(180));
   public static Pose2d intermediate = new Pose2d(-50, 54, Math.toRadians(-90));

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
      driveToScoring();
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

   private void driveToScoring() {
      sched.addAction(
              drive.actionBuilder(spike[SPIKE])
                      .strafeToLinearHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                      .strafeToLinearHeading(intermediate.position, intermediate.heading)
                      .splineTo(new Vector2d(intermediate.position.x, parking.position.y), intermediate.heading)
                      .splineTo(new Vector2d((intermediate.position.x + AutoConstants.blueScoring[SPIKE].position.x * 2)/3, parking.position.y), AutoConstants.blueScoring[SPIKE].heading)
                      .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                      .build()
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(AutoConstants.blueScoring[SPIKE])
                              .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.extendOuttakeMidBlocking()
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
