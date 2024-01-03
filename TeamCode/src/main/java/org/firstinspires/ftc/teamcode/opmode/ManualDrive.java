package org.firstinspires.ftc.teamcode.opmode;

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
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
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
   private Hang hang;
   private Plane plane;
   private Vision vision;
   private LED led;
   private BeamBreakSensor sensor;

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
      hang = new Hang(hardwareMap);
      plane = new Plane(hardwareMap);
      vision = new Vision(hardwareMap);
      led = new LED(hardwareMap);
      sensor = new BeamBreakSensor(hardwareMap, "intakeBeam");

      if (Memory.RAN_AUTO) {
         smartGameTimer = new SmartGameTimer(true);
      } else { // No auto memory, pull in slides
         smartGameTimer = new SmartGameTimer(false);
         outtake.prepTeleop();
      }

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
         outtake.initialize();
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
         hang.update();

         telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
         telemetry.addData("Beam Broken", sensor.isBeamBroken());
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
      // Vision dist not possible due to camera not being able to focus when up against backdrop
      /*double visionDist = vision.backdropDistance(50); // Refresh every 50 ticks unless something is within range, then 5
      if (visionDist < VISION_CLOSE_DIST + VISION_RANGE) {
         visionDist = vision.backdropDistance(5);
      }
      telemetry.addData("VISION DISTANCE", visionDist);*/
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
      if (g2.leftBumper()) input_turn += D2_SLOW_TURN;
      if (g2.rightBumper()) input_turn -= D2_SLOW_TURN;

      drive.setDrivePowers(new PoseVelocity2d(input, input_turn));
   }

   private void intakeControls() {
      if (intake.hit2) {
         intake.hit2 = false;
         sched.queueAction(
                 new SequentialAction(
                         intake.intakeOff(),
                         outtake.latchClosed()
                 )
         );
      }

      // Intake controls
      if (g1.aOnce()) {
         if (intake.intakeState == Intake.IntakeState.On) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(outtake.latchClosed());
         } else {
            sched.queueAction(intake.intakeOn());
            sched.queueAction(outtake.latchOpen());
         }
      }
      if (g1.b()) {
         if (intake.intakeState == Intake.IntakeState.On) {
            sched.queueAction(outtake.latchClosed());
         }
         sched.queueAction(intake.intakeReverse());
      }
      if (!g1.b() && intake.intakeState == Intake.IntakeState.Reversing) {
         sched.queueAction(intake.intakeOff());
      }

      // Other susbsystems
      if (g1.dpadUpOnce()) {
         sched.queueAction(hang.extendHang());
      }
      if (g1.dpadDownOnce()) {
         sched.queueAction(hang.retractHang());
      }
      if (g1.backOnce()) {
         sched.queueAction(plane.scorePlane());
      }
      if (g1.startOnce()) {
         sched.queueAction(new SequentialAction(
                 intake.stackClosed(),
                 new SleepAction(0.6),
                 intake.stackOpen(),
                 new SleepAction(0.6)
         ));
      }
      if (g2.startOnce()) {
         // The servo will rotate the beam so the driver can adjust pixels for mosaic creation.
         sched.queueAction(outtake.mosaicAdjust());
      }

      if (g2.backOnce()) {
         // The the servo will rotate the beam so it is safely inside the robot.
         sched.queueAction(outtake.mosaicClosed());
      }
      if (g2.dpadLeft()) {
         sched.queueAction(outtake.increaseMosaicPos());
      }
      if (g2.dpadRight()) {
         sched.queueAction(outtake.decreaseMosaicPos());
      }
   }

   private void outtakeControls() {
      // Outtake controls
      if (g2.yOnce()) {
         sched.queueAction(intake.intakeOff());
         sched.queueAction(new SequentialAction(outtake.latchClosed(), new SleepAction(0.1)));
         sched.queueAction(new ParallelAction(
                 new SequentialAction(new SleepAction(0.4), outtake.wristScoring()),
                 outtake.extendOuttakeTeleopBlocking()
         ));
         intake.pixelCount = 2; // If the outtake slide's are extended and ready for deposit we probably have 2 pixels.
         //Since the beam breaks aren't finalized yet I'm using this just to test whether the outtake can release one by one.
      }
      if (g2.xOnce()) {
         if (intake.pixelCount >= 2) {
            sched.queueAction(outtake.latchScoring());
            sched.queueAction(intake.setPixelCount(1));
         } else {
            sched.queueAction(outtake.latchOpen());
            sched.queueAction(intake.setPixelCount(0));
         }
      }
      if (Math.abs(g2.right_stick_y) > 0.01) {
         outtake.slidePIDEnabled = false;
         outtake.setSlidePower(-g2.right_stick_y);
      } else if (!outtake.slidePIDEnabled) {
         outtake.slidePIDEnabled = true;
         outtake.lockPosition();
      }
      if (g2.bOnce()) {
         sched.queueAction(outtake.wristStored());
         new SleepAction(1);
         sched.queueAction(outtake.retractOuttake());
         sched.queueAction(outtake.latchClosed());
      }
      if (g2.dpadUpOnce()) {
         sched.queueAction(outtake.latchScoring());
      }
      if (g2.dpadDownOnce()) {
         if (intake.intakeState == Intake.IntakeState.On) {
            sched.queueAction(outtake.latchOpen());
         } else {
            sched.queueAction(outtake.latchClosed());
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
      if (intake.pixelCount == 1) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
      } else if (intake.pixelCount == 2) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
      } else if (intake.pixelCount > 2) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
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
