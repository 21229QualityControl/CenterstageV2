package org.firstinspires.ftc.teamcode.opmode.auto.legacy;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

import java.util.Vector;

@Config
//@Disabled
@Autonomous(name = "Red Left Auto", group = "Auto", preselectTeleOp = "Manual Drive")
public class RedLeftAuto extends AutoBase {
   public static Pose2d[] spike = {new Pose2d(-28, -37, Math.toRadians(-135)), new Pose2d(-36, -33, Math.toRadians(-90)), new Pose2d(-48, -39, Math.toRadians(-90))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {new Pose2d(-39, -45, Math.toRadians(-135)), new Pose2d(-36, -41, Math.toRadians(-90)), new Pose2d(-48, -47, Math.toRadians(-90))};
   public static Pose2d start = new Pose2d(-36, -63, Math.toRadians(-90));
   public static Pose2d parking = new Pose2d(60, -10, Math.toRadians(180));
   public static Pose2d intermediate = new Pose2d(-40, -59, Math.toRadians(180));

   @Override
   protected Pose2d getStartPose() {
      return start;
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Red Left Auto");
   }

   @Override
   protected void onRun() {
      sched.addAction(new SleepAction(10));
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
                      outtake.extendOuttakeTeleopBlocking(),
                      drive.actionBuilder(AutoConstants.redScoring[SPIKE])
                              .afterDisp(0, new ActionUtil.RunnableAction(() -> {
                                 double dist = frontSensors.backdropDistance();
                                 if (dist > 15) {
                                    dist = 9; // failed
                                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                                 }
                                 Log.d("BACKDROP_DIST", String.valueOf(dist));
                                 drive.pose = new Pose2d(drive.pose.position.plus(new Vector2d(13 - dist, 0)), drive.pose.heading);
                                 drive.updatePoseEstimate();
                                 return false;
                              }))
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
