package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoConstants;

@Config
@Autonomous(name = "Blue Left Auto 2+4", group = "Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftAutoSpline extends AutoBase {
   public static Pose2d[] spike = {new Pose2d(4, 36, Math.toRadians(45)), new Pose2d(12, 33, Math.toRadians(90)), new Pose2d(24, 42, Math.toRadians(90))};
   // 0 = right, 1 = middle, 2 = left
   public static Pose2d[] spikeBackedOut =  {new Pose2d(17, 46, Math.toRadians(45)), new Pose2d(12, 44, Math.toRadians(90)), new Pose2d(24, 55, Math.toRadians(90))};
   public static Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
   public static Pose2d parking = new Pose2d(56, 60, Math.toRadians(180));
   public static Pose2d[] stackPositions = {new Pose2d( -68, 13, Math.toRadians(180)), new Pose2d(-68, 13, Math.toRadians(180)), new Pose2d(-68, 14, Math.toRadians(180))}; // TODO: Figure out why it has to be different based on spike
   private Pose2d stack;

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
      stack = stackPositions[SPIKE];
      deliverSpike();
      scorePreload();
      intakeStack(true);
      scoreStack();
      intakeStack(false);
      scoreStack();
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
   }

   private void scorePreload() {
         sched.addAction(
                 drive.actionBuilder(spike[SPIKE])
                         .splineToConstantHeading(spikeBackedOut[SPIKE].position, spikeBackedOut[SPIKE].heading)
                         .splineToSplineHeading(AutoConstants.blueScoring[SPIKE], AutoConstants.blueScoring[SPIKE].heading)
                         .afterDisp(0, new SequentialAction(
                                 outtake.wristScoring(),
                                 outtake.extendOuttakeLowBlocking()
                         ))
                         .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading)
                         .build()
         );
         sched.addAction(
                 new SequentialAction(
                         outtake.latchOpen(),
                         new SleepAction(0.3),
                         outtake.extendOuttakeMid(),
                         new SleepAction(0.3)
                 )
         );
   }

   private void intakeStack(boolean firstCycle) {
      sched.addAction(
              drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                      .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                      .afterDisp(0, new SequentialAction(
                              outtake.extendOuttakeMidBlocking(),
                              outtake.wristStored(),
                              new SleepAction(0.3),
                              outtake.retractOuttake(),
                              outtake.latchOpen(),
                              new SleepAction(0.3)
                      ))
                      .splineToConstantHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x, stack.position.y), stack.heading)
                      .afterDisp(0, new SequentialAction(
                              intake.intakeOn(),
                              intake.stackHalf()
                      ))
                      .splineToConstantHeading(stack.position, stack.heading)
                      .afterDisp(0, new SequentialAction(
                              intake.stackClosed(),
                              new SleepAction(0.3),
                              intake.stackHalf()
                              // continues in scoreStack()
                      ))
                      .build()
      );
      if (firstCycle) {
         SPIKE = (SPIKE + 1) % 3;
      }
   }

   private void scoreStack() {
      sched.addAction(
              drive.actionBuilder(stack)
                      .setReversed(true)
                      .afterTime(0, new SequentialAction(
                              // starts in intakeStack()
                              new SleepAction(0.3),
                              intake.stackClosed(),
                              new SleepAction(0.3),
                              intake.stackHalf()
                      ))
                      .splineToConstantHeading(new Vector2d(AutoConstants.blueScoring[SPIKE].position.x - 12, stack.position.y), stack.heading)
                      .afterDisp(0, new SequentialAction(
                              intake.intakeOff(),
                              outtake.latchClosed(),
                              intake.stackOpen()
                      ))
                      .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position, AutoConstants.blueScoring[SPIKE].heading)
                      .afterDisp(0, new SequentialAction(
                              outtake.extendOuttakeTeleopBlocking(),
                              new SleepAction(0.1),
                              outtake.wristScoring()
                      ))
                      .splineToConstantHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading)
                      .afterDisp(0, new SequentialAction(
                              outtake.latchOpen(),
                              outtake.extendOuttakeMid(),
                              new SleepAction(0.3)
                      ))
                      .build()
      );
   }

   private void park() {
      sched.addAction(
           drive.actionBuilder(new Pose2d(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(10, 0)), AutoConstants.blueScoring[SPIKE].heading))
                   .strafeToLinearHeading(AutoConstants.blueScoring[SPIKE].position.plus(new Vector2d(5, 0)), AutoConstants.blueScoring[SPIKE].heading)
                   .afterDisp(0, new SequentialAction(
                           outtake.wristStored(),
                           outtake.latchClosed(),
                           outtake.retractOuttake()
                   ))
                   .build()
      );
   }
}
