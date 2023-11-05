package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;

@Config
@TeleOp(group = "Drive")
public class ManualDrive extends LinearOpMode {
   public static double TURN_SPEED = 0.75;
   public static double DRIVE_SPEED = 1;
   public static double SLOW_TURN_SPEED = 0.3;
   public static double SLOW_DRIVE_SPEED = 0.3;

   private SmartGameTimer smartGameTimer;
   private GamePadController g1, g2;
   private MecanumDrive drive;
   private ActionScheduler sched;
   private Intake intake;
   private Outtake outtake;
   private Hang hang;
   private Plane plane;

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
      }

      // Main loop
      while (opModeIsActive()) {
         g1.update();
         g2.update();

         move();
         subsystemControls();

         drive.updatePoseEstimate();
         sched.update();
         outtake.update();
         intake.update();
         hang.update();

         telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
         telemetry.update();
      }

      // On termination
      Memory.LAST_POSE = drive.pose;
   }

   private void move() {
      double speed = (1-Math.abs(g1.right_stick_x)) * (DRIVE_SPEED - SLOW_DRIVE_SPEED) + SLOW_DRIVE_SPEED;
      double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
      double input_y = Math.pow(-g1.left_stick_x, 3) * speed;
      double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;

      if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
      if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

      drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input_x, input_y), input_turn));
   }

   boolean slideHigh = false;
   private void subsystemControls() {
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
         if (slideHigh) {
            slideHigh = false;
            sched.queueAction(outtake.retractOuttake());
            sched.queueAction(outtake.wristStored());
            sched.queueAction(outtake.latchClosed());
         }
         if (intake.intakeState == Intake.IntakeState.On) {
            sched.queueAction(outtake.latchClosed());
         }
         sched.queueAction(intake.intakeReverse());
      }
      if (!g1.b() && intake.intakeState == Intake.IntakeState.Reversing) {
         sched.queueAction(intake.intakeOff());
      }

      // Outtake controls
      if (g1.yOnce()) {
         if (slideHigh) {
            sched.queueAction(new SequentialAction(
                    outtake.wristHolding(),
                    new SleepAction(0.5),
                    outtake.latchOpen(),
                    new SleepAction(0.5),
                    outtake.latchClosed(),
                    new SleepAction(0.3),
                    outtake.wristScoring()
            ));
         } else {
            slideHigh = true;
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(outtake.latchClosed(), new SleepAction(0.1)));
            sched.queueAction(new ParallelAction(
                    new SequentialAction(new SleepAction(0.4), outtake.wristScoring()),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         }
      }
      if (g1.xOnce()) {
         sched.queueAction(outtake.latchScoring());
      }
      if (Math.abs(g1.right_stick_y) > 0.01  && slideHigh) {
         outtake.slidePIDEnabled = false;
         outtake.setSlidePower(-g1.right_stick_y);
      } else if (!outtake.slidePIDEnabled) {
         outtake.slidePIDEnabled = true;
         outtake.lockPosition();
      }

      // Other subsystems
      if (g1.dpadUpOnce()) {
         sched.queueAction(hang.extendHang());
      }
      if (g1.dpadDownOnce()) {
         sched.queueAction(hang.retractHang());
      }
      if (g1.startOnce()) {
         sched.queueAction(plane.scorePlane());
      }
   }
}
