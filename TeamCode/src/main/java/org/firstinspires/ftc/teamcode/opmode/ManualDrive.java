package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.LED;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

@Config
@TeleOp(group = "Drive")
public class ManualDrive extends LinearOpMode {
   public static double TURN_SPEED = 0.75;
   public static double DRIVE_SPEED = 1;
   public static double SLOW_TURN_SPEED = 0.3;
   public static double D2_SLOW_TURN = 0.25;
   public static double SLOW_DRIVE_SPEED = 0.3;
   public static double VISION_RANGE = 20;
   public static double VISION_CLOSE_DIST = 5;

   private SmartGameTimer smartGameTimer;
   private GamePadController g1, g2;
   private MecanumDrive drive;
   private ActionScheduler sched;
   private Intake intake;
   private Outtake outtake;
   private Plane plane;
   private LED led;
   private long lastLoopFinish = 0;

   public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
   private PIDFController headingPid;

   @Override
   public void runOpMode() throws InterruptedException {
      telemetry.addLine("Initializing...");
      telemetry.update();

      // Init
      g1 = new GamePadController(gamepad1);
      g2 = new GamePadController(gamepad2);
      g1.update();
      g2.update();
      sched = new ActionScheduler();
      drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
      intake = new Intake(hardwareMap);
      outtake = new Outtake(hardwareMap);
      plane = new Plane(hardwareMap);
      led = new LED(hardwareMap);
      headingPid = new PIDFController(HEADING_PID);

      if (Memory.RAN_AUTO) {
         smartGameTimer = new SmartGameTimer(true);
      } else { // No auto memory, pull in slides
         smartGameTimer = new SmartGameTimer(false);
         outtake.prepInitializeSlides();
         telemetry.addLine("Initializing slides...");
         telemetry.update();
         sleep(200);
         while (opModeInInit() && outtake.initializeSlides()) {}
      }
      led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

      // Ready!
      telemetry.addLine("Ready!");
      telemetry.update();
      waitForStart();

      // Opmode start
      if (opModeIsActive()) {
         resetRuntime();
         g1.reset();
         g2.reset();

         // Init opmodes
         outtake.initialize(true);
         plane.initialize();
         intake.initialize();
         smartGameTimer.resetIfStandard();
      }

      // Main loop
      while (opModeIsActive()) {
         g1.update();
         g2.update();

         move();
         intakeControls();
         outtakeControls();
         ledUpdate();

         drive.updatePoseEstimate();
         sched.update();
         outtake.update();
         intake.update();

         telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
         telemetry.addData("Pixel Count", intake.pixelCount());

         long finish = System.currentTimeMillis();
         telemetry.addData("Sample Rate (Hz) ",1/((double)(finish - lastLoopFinish)/1000.0));
         telemetry.update();

         lastLoopFinish = finish;
      }

      // On termination
      Memory.LAST_POSE = drive.pose;
   }

   private double prevInputX = 0;
   private void move() {
      // Main driver controls
      double visionDist = 10000;
      double speed = Math.min(1, Math.max(0, ((visionDist - VISION_CLOSE_DIST) / VISION_RANGE))) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;

      //ATTEMPT 1
      //      double input_x = Math.pow(-g1.left_stick_y * (intake.isIntakeNearWall() ? SLOW_DRIVE_SPEED: 1),
//              3) * speed;
//      double input_y = Math.pow(-g1.left_stick_x * (intake.isIntakeNearWall() ? SLOW_DRIVE_SPEED: 1),
//              3) * speed;

      //ATTEMPT 2
//      double input_x = Math.pow(-g1.left_stick_y * (intake.isIntakeNearWall() ? Range.clip((intake.getDistance()-200)/500, 0.15, 0.5) : 1), 3) * speed;
//      input_x = (input_x >= 0) ? Range.clip(input_x,(input_x/Math.abs(input_x))*0.15,(input_x/Math.abs(input_x))*0.5): input_x;
//      double input_y = Math.pow(-g1.left_stick_x * (intake.isIntakeNearWall() ? Range.clip((intake.getDistance()-200)/500, 0.15, 0.5) : 1), 3) * speed;
//      input_y = (input_y >= 0) ? Range.clip(input_y,(input_y/Math.abs(input_y))*0.15,(input_y/Math.abs(input_y))*0.5): input_y;

      /*
      // SLOWMODE
      if (!outtake.isSlideRetracted()) {
         g2.left_stick_x -= g1.left_stick_x;
         g2.left_stick_y -= g1.left_stick_y;
         g1.left_stick_x = 0;
         g1.left_stick_y = 0;
      }*/

      double input_x;
      double input_y;
      /*if (-g1.left_stick_y > 0 && intake.isIntaking()) {
         input_x = Math.pow(-g1.left_stick_y * (intake.isIntakeNearWall() ? Range.clip((intake.getDistance()-200)/800, 0.4, 0.8) : 1), 3) * speed;
         input_x = Range.clip(input_x, 0.15, 1);
        if (intake.willIntakeHitWall()) {
            input_x = 0;
         } // Messed up when intaking in corner
      } else {
         input_x = Math.pow(-g1.left_stick_y, 3) * speed;
      }*/
      input_x = Math.pow(-g1.left_stick_y, 3) * speed;
      input_y = Math.pow(-g1.left_stick_x, 3) * speed;

      Vector2d input = new Vector2d(input_x, input_y);
      //input = drive.pose.heading.inverse().times(input); // Field centric

      double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
      if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
      if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

      // Driver 2 slow strafe
      input = input.plus(new Vector2d(g2.left_stick_y * SLOW_DRIVE_SPEED, g2.left_stick_x * SLOW_DRIVE_SPEED));
      input_turn += g2.left_trigger * D2_SLOW_TURN;
      input_turn -= g2.right_trigger * D2_SLOW_TURN;

      if (Math.abs(g2.left_stick_x) != 0 && Math.abs(prevInputX) < EPSILON) {
         headingPid.setTargetPosition(drive.pose.heading.toDouble());
      }
      prevInputX = g2.left_stick_x;
      if (Math.abs(input_turn) > EPSILON) {
         prevInputX = 0;
      }
      if (Math.abs(g1.left_stick_x + g1.left_stick_y + input_turn) < EPSILON && Math.abs(g2.left_stick_x) > 0) { // Do heading lock
         input_turn = headingPid.update(drive.pose.heading.toDouble());
         if (g2.left_stick_x > 0) { // Account for heading turn overpowering strafe
            input = input.plus(new Vector2d(0, Math.abs(input_turn)));
         } else {
            input = input.plus(new Vector2d(0, -Math.abs(input_turn)));
         }
      }

      drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
   }

   private void intakeControls() {
      // Intake controls
      if (intake.isIntakeOverCurrent()) {
         sched.queueActionParallel(intake.wristStored());
         long start = System.currentTimeMillis();
         sched.queueActionParallel(new ActionUtil.RunnableAction(() -> {
            Log.d("CURRENT", intake.isIntakeOverCurrent() ? "true" : "false");
            if (!intake.isIntakeOverCurrent() || intake.pixelCount() == 2) {
               sched.queueAction(intake.wristDown());
               sched.queueAction(intake.intakeOn());
               return false;
            }
            if (System.currentTimeMillis() - start > 1000 && !intake.isReversing()) {
               sched.queueAction(intake.intakeReverse());
            }
            return true;
         }));
      }

      int pixelCount = intake.pixelCount();
      if (intake.isIntaking() && pixelCount == 2) { // Check if already two pixels - Stop intake
         sched.queueAction(intake.intakeReverse());
         sched.queueAction(intake.wristStored());
         sched.queueActionParallel(new SequentialAction(
                 new SleepAction(1),
                 intake.intakeOff(),
                 intake.wristStored()
         ));
      }
      if (g1.aOnce()) {
         sched.cancelParallel();
         if (intake.isIntaking()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(intake.wristStored());
         } else {
            sched.queueAction(intake.intakeOn());
            sched.queueAction(intake.wristDown());
         }
      }
      if (g1.bOnce()) {
         sched.cancelParallel();
         if (intake.isReversing()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(intake.wristStored());
         } else {
            sched.queueAction(intake.intakeReverse());
         }
      }
      if (g1.xOnce()) {
         sched.queueAction(new SequentialAction(
                 intake.feedOpen(),
                 new SleepAction(0.5),
                 intake.feedClosed()
         ));
      }
      if (g1.dpadUpOnce()) {
         sched.cancelParallel();
         sched.queueAction(intake.prepIntakeCount(true, pixelCount == 1));
         sched.queueActionParallel(new SequentialAction(
                 intake.intakeCount(false),
                 new SleepAction(1),
                 intake.intakeOff()
         ));
      }
      if (g1.dpadDownOnce()) {
         sched.cancelParallel();
         sched.queueAction(intake.prepIntakeCount(false, pixelCount == 1));
         sched.queueActionParallel(new SequentialAction(
                 intake.intakeCount(false),
                 new SleepAction(1),
                 intake.intakeOff()
         ));
      }

      // Other susbsystems
      if (g1.backOnce()) {
         sched.queueAction(plane.scorePlane());
      }
      if (g2.startOnce()) {
         if (outtake.isSlideRetracted()) {
            sched.queueAction(outtake.extendOuttakeHangBlocking());
            // Make mosaic fixer go out to not interfere with hanging
            sched.queueAction(outtake.mosaicFix());
         } else {
            sched.queueAction(outtake.retractOuttakeHang());
         }
      }
   }

   private void outtakeControls() {
      // Outtake controls
      if (g2.yOnce()) {
         if (!outtake.isArmScoring()) {
            if (intake.pixelCount() == 1) {
               sched.queueAction(new SequentialAction(outtake.extendOuttakeBarelyOut(), new SleepAction(0.3)));
            }
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(
                    intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                    new SleepAction(0.3)
           ));
            sched.queueAction(new ParallelAction(
                    new SequentialAction(new SleepAction(0.4), outtake.armScoring()),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         }
         sched.queueAction(outtake.wristVertical());
      }
      if (g2.xOnce()) {
         if (!outtake.isArmScoring()) {
            if (intake.pixelCount() == 1) {
               sched.queueAction(new SequentialAction(outtake.extendOuttakeBarelyOut(), new SleepAction(0.3)));
            }
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(
                    intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                    new SleepAction(0.3)
            ));
            sched.queueAction(new ParallelAction(
                    new SequentialAction(new SleepAction(0.4), outtake.armScoring()),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         }
         sched.queueAction(outtake.wristMosaic(true));
      }

      if (g2.leftBumperOnce()) {
         if (outtake.isWristVertical()) {
            sched.queueAction(outtake.wristSideways(true));
         } else if (outtake.isWristSideways()) {
            sched.queueAction(outtake.wristVerticalFlip());
         }

         if (outtake.isWristMosaic(true)) {
            sched.queueAction(outtake.wristMosaicFlip(true));
         } else if (outtake.isWristMosaic(false)) {
            sched.queueAction(outtake.wristMosaic(true));
         }
      }

      if (g2.rightBumperOnce()) {
         if (outtake.isWristVertical()) {
            sched.queueAction(outtake.wristSideways(false));
         }
         if (outtake.isWristSideways()) {
            sched.queueAction(outtake.wristVerticalFlip());
         }

         if (outtake.isWristMosaic(false)) {
            sched.queueAction(outtake.wristMosaicFlip(false));
         } else if (outtake.isWristMosaic(true)) {
            sched.queueAction(outtake.wristMosaic(false));
         }
      }

      if (Math.abs(g2.right_stick_y) > 0.01) {
         outtake.slidePIDEnabled = false;
         outtake.setSlidePower(-g2.right_stick_y/1.6);
      } else if (!outtake.slidePIDEnabled) {
         outtake.slidePIDEnabled = true;
         sched.queueAction(outtake.lockPosition());
      }
      if (g2.aOnce()) {
         sched.queueAction(new SequentialAction(
                 outtake.mosaicStored(),
                 outtake.clawHalfOpen(),
                 new SleepAction(outtake.isArmScoring() ? 0.5 : 0),
                 outtake.wristVertical(),
                 outtake.armStored(),
                 outtake.clawOpen(),
                 outtake.retractOuttakeBlocking()
         ));
      }
      if (g2.bOnce()) {
         sched.queueAction(new SequentialAction(
                 outtake.wristVertical(),
                 outtake.armStored(),
                 new SleepAction(0.3),
                 outtake.retractOuttakeBlocking(),
                 new SleepAction(0.1),
                 outtake.clawOpen()
         ));
      }
      if (g2.dpadUpOnce()) {
         sched.queueAction(outtake.increaseSlideLayer(1));
      }
      if (g2.dpadDownOnce()) {
         sched.queueAction(outtake.increaseSlideLayer(-1));
      }

      // Mosaic controls
      if (g2.startOnce()) { // Press start to mosaic adjust with outtake
         sched.queueAction(outtake.clawClosed());
      }

      // Press back to toggle between mosaic fix and mosaic stored.
      if (g2.backOnce()) {
         if (outtake.isMosaicFixing()) {
            sched.queueAction(outtake.mosaicStored());
         } else {
            sched.queueAction(outtake.mosaicFix());
            sched.queueAction(outtake.wristVertical());
            sched.queueAction(outtake.armStored());
            sched.queueAction(outtake.clawOpen());
         }
      }
   }

   private double timeLeft() {
      if (!isStarted()) return 120;
      return 120 - smartGameTimer.seconds();
   }
   private boolean isBetween(double number, double min, double max) {
      return min <= number && number < max;
   }
   boolean warning1 = false;
   boolean warning2 = false;
   boolean warning3 = false;
   private void ledUpdate() {
      int pixelCount = intake.pixelCount();
      if (pixelCount == 1) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
      } else if (pixelCount == 2) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
      } else if (isBetween(timeLeft(), 31, 35)) { // 35-31; prepare for endgame
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
         if (!warning1) {
            g1.rumbleBlips(3);
            warning1 = true;
         }
      } else if (isBetween(timeLeft(), 25, 30)) { // 30-25; endgame starts
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
         if (!warning2) {
            g1.rumbleBlips(3);
            warning2 = true;
         }
      } else if (isBetween(timeLeft(), 0, 5)) { // last 5 sec, go park
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
         if (!warning3) {
            g1.rumbleBlips(3);
            warning3 = true;
         }
      } else { // default lights, put here for lower priority
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
      }
   }
}
