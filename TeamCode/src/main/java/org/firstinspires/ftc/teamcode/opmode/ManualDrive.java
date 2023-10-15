package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
         outtake.initializeTeleop();
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
         hang.update();

         telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
         telemetry.update();
      }

      // On termination
      Memory.LAST_POSE = drive.pose;
   }

   private void move() {
      double input_x = Math.pow(-g1.left_stick_y, 3) * DRIVE_SPEED;
      double input_y = Math.pow(-g1.left_stick_x, 3) * DRIVE_SPEED;
      double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;

      if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
      if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

      drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input_x, input_y), input_turn));
   }

   private void subsystemControls() {
      // Intake controls
      if (g1.a() && !intake.isIntakeOn()) {
         sched.queueAction(intake.intakeOn());
      }
      if (!g1.a() && intake.isIntakeOn()) {
         sched.queueAction(intake.intakeOff());
      }

      // Outtake controls
      if (g1.yOnce()) {
         sched.queueAction(new SequentialAction(outtake.latchClosed(), new SleepAction(0.1)));
         sched.queueAction(new ParallelAction(
                 new SequentialAction(new SleepAction(0.2), outtake.wristScoring()),
                 outtake.extendOuttakeBlocking()
         ));
      }
      if (g1.xOnce()) {
         sched.queueAction(outtake.latchScoring());
      }
      if (g1.bOnce()) {
         sched.queueAction(outtake.latchOpen());
         sched.queueAction(outtake.wristStored());
         sched.queueAction(outtake.retractOuttake());
      }
   }
}
