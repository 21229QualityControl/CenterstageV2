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
@Autonomous(name = "Red Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends AutoBase {
   public static Pose2d[] spike = {new Pose2d(-33, -40, Math.toRadians(-135)), new Pose2d(-36, -38, Math.toRadians(-90)), new Pose2d(-48, -44, Math.toRadians(-90))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {new Pose2d(-41, -50, Math.toRadians(-135)), new Pose2d(-36, -49, Math.toRadians(-90)), new Pose2d(-48, -50, Math.toRadians(-90))};
   public static Pose2d start = new Pose2d(-36, -64, Math.toRadians(-90));
   public static Pose2d parking = new Pose2d(56, -12, Math.toRadians(180));
   public static Pose2d intermediate = new Pose2d(-40, -56, Math.toRadians(180));

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
                      .strafeToLinearHeading(new Vector2d((AutoConstants.redScoring[SPIKE].position.x*2 + intermediate.position.x)/3, intermediate.position.y), AutoConstants.redScoring[SPIKE].heading)
                      .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                      .build()
      );
   }

   private void scorePreload() {
      sched.addAction(
              new SequentialAction(
                      outtake.wristScoring(),
                      outtake.extendOuttakeLowBlocking(),
                      drive.actionBuilder(AutoConstants.redScoring[SPIKE])
                              .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.redScoring[SPIKE].heading) // Correct for any turning that occured during the previous move
                              .build(),
                      outtake.latchScoring(),
                      new SleepAction(0.5),
                      outtake.extendOuttakeMidBlocking()
              )
      );
   }

   private void park() {
      sched.addAction(
                   drive.actionBuilder(new Pose2d(AutoConstants.redScoring[SPIKE].position.plus(new Vector2d(10, 0)),
                                   AutoConstants.redScoring[SPIKE].heading))
                           .strafeToLinearHeading(AutoConstants.redScoring[SPIKE].position, AutoConstants.redScoring[SPIKE].heading)
                           .afterDisp(10, new SequentialAction(
                                   outtake.wristStored(),
                                   new SleepAction(0.5),
                                   outtake.retractOuttake(),
                                   outtake.latchClosed(),
                                   new SleepAction(0.5)
                           ))
                           .strafeToLinearHeading(new Vector2d(AutoConstants.redScoring[SPIKE].position.x, parking.position.y), parking.heading)
                           .strafeToLinearHeading(parking.position, parking.heading)
                           .build()
      );
   }
}
