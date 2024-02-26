package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.LED;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;

@Config
@TeleOp(group = "Drive")
public class ManualDrive extends LinearOpMode {
   public static double TURN_SPEED = 0.75;
   public static double DRIVE_SPEED = 1;
   public static double SLOW_TURN_SPEED = 0.3;
   public static double D2_SLOW_TURN = 0.2;
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

      if (Memory.RAN_AUTO) {
         smartGameTimer = new SmartGameTimer(true);
      } else { // No auto memory, pull in slides
         smartGameTimer = new SmartGameTimer(false);
         outtake.prepTeleop();
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

         // Finish pulling in slides
         if (!smartGameTimer.isNominal()) {
            outtake.finishPrepTeleop();
         }

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
         telemetry.update();

         telemetry.update();
         telemetry.update();
      }

      // On termination
      Memory.LAST_POSE = drive.pose;
   }

   private void move() {
      if (g1.backOnce()) { // Reset field centric
         drive.pose = new Pose2d(0, 0, Math.toRadians(180));
         drive.imu.resetYaw();
      }

      // Main driver controls
      double visionDist = 10000;
      double speed = Math.min(1, Math.max(0, ((visionDist - VISION_CLOSE_DIST) / VISION_RANGE))) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;
      double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
      double input_y = Math.pow(-g1.left_stick_x, 3) * speed;
      Vector2d input = new Vector2d(input_x, input_y);
      //input = drive.pose.heading.inverse().times(input); // Field centric

      double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
      if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
      if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

      // Driver 2 slow strafe
      input = input.plus(new Vector2d(g2.left_stick_y * SLOW_DRIVE_SPEED, g2.left_stick_x * SLOW_DRIVE_SPEED));
      input_turn += g2.left_trigger * D2_SLOW_TURN;
      input_turn -= g2.right_trigger * D2_SLOW_TURN;

      drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
   }

   public int INTAKE_STACK_POSITION = 0;
   private void intakeControls() {
      /*if (intake.isIntaking() && intake.pixelCount() == 2) {
          sched.queueAction(intake.intakeOff()); TODO: Enable this
      }*/

      // Intake controls
      if (g1.aOnce()) {
         if (intake.isIntaking()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(intake.wristStored());
         } else {
            sched.queueAction(intake.intakeOn());
            sched.queueAction(intake.wristDown());
         }
      }
      if (g1.b()) {
         sched.queueAction(intake.intakeReverse());
      }
      if (!g1.b() && intake.isReversing()) {
         sched.queueAction(intake.intakeOff());
      }
      if (g1.dpadUpOnce()) {
         INTAKE_STACK_POSITION--;
         if (INTAKE_STACK_POSITION < 0) {
            INTAKE_STACK_POSITION = 0;
            sched.queueAction(intake.wristStored());
            sched.queueAction(intake.intakeOff());
         } else {
            sched.queueAction(intake.wristStack(INTAKE_STACK_POSITION));
         }
         Log.d("STACKPOSITION", String.valueOf(INTAKE_STACK_POSITION));
      }
      if (g1.dpadDownOnce()) {
         if (intake.wristIsStored()) {
            INTAKE_STACK_POSITION = 0; // Reset
            sched.queueAction(intake.intakeOn());
         } else {
            INTAKE_STACK_POSITION++;
         }
         if (INTAKE_STACK_POSITION > 3) {
            INTAKE_STACK_POSITION = 0;
            sched.queueAction(intake.wristDown());
         } else {
            sched.queueAction(intake.wristStack(INTAKE_STACK_POSITION));
         }
         Log.d("STACKPOSITION", String.valueOf(INTAKE_STACK_POSITION));
      }
      if (g1.dpadLeftOnce()) {
         INTAKE_STACK_POSITION = 0;
         sched.queueAction(intake.wristStored());
      }
      if (g1.dpadRightOnce()) {
         sched.queueAction(intake.wristDown());
      }

      // Other susbsystems
      if (g1.backOnce()) {
         sched.queueAction(plane.scorePlane());
      }
      if (g1.startOnce()) {
         if (outtake.isSlideRetracted()) {
            sched.queueAction(outtake.extendOuttakeHangBlocking());
         } else {
            sched.queueAction(outtake.retractOuttake());
         }
      }
   }

   private void outtakeControls() {
      // Outtake controls
      if (g2.yOnce()) {
         if (!outtake.isArmScoring()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(
                    outtake.clawClosed(),
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
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(outtake.clawClosed(), new SleepAction(0.3)));
            sched.queueAction(new ParallelAction(
                    new SequentialAction(new SleepAction(0.4), outtake.armScoring()),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         }
         sched.queueAction(outtake.wristMosaic(true));
      }

      if (g2.leftBumperOnce()) {
         if (outtake.isWristMosaic()) {
            sched.queueAction(outtake.wristMosaic(true));
         } else {
            sched.queueAction(outtake.wristSideways(true));
         }
      }

      if (g2.rightBumperOnce()) {
         if (outtake.isWristMosaic()) {
            sched.queueAction(outtake.wristMosaic(false));
         } else {
            sched.queueAction(outtake.wristSideways(false));
         }
      }

      if (g2.aOnce()) {
         sched.queueAction(new SequentialAction(
                 intake.feedOpen(),
                 new SleepAction(0.5),
                 intake.feedClosed()
         ));
      }

      if (Math.abs(g2.right_stick_y) > 0.01) {
         outtake.slidePIDEnabled = false;
         outtake.setSlidePower(-g2.right_stick_y);
      } else if (!outtake.slidePIDEnabled) {
         outtake.slidePIDEnabled = true;
         sched.queueAction(outtake.lockPosition());
      }
      if (g2.bOnce()) {
         sched.queueAction(new SequentialAction(
                 outtake.armStored(),
                 outtake.wristVertical(),
                 outtake.retractOuttakeBlocking(),
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
