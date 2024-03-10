package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 800; // Max speed is 2400
   public static int INTAKE_REVERSE_SPEED = -2400;

   final MotorWithVelocityPID intakeMotor;
   final Servo intakeWristLeft;
   final Servo intakeWristRight;
   final Servo intakeFeed;

   final BeamBreakSensor beamBreak;

   public static double WRIST_LEFT_STORED = 0.17;
   public static double WRIST_LEFT_DOWN = 0.43;
   public static double[] WRIST_LEFT_STACK_POSITIONS = {
           0.32, // Getting 1
           0.35, // Getting 2
           0.38, // Getting 3
           0.41, // Getting 4
           // Use WRIST_DOWN to get 5
   };

   public static double WRIST_RIGHT_STORED = 0.34;
   public static double WRIST_RIGHT_DOWN = 0.09;
   public static double[] WRIST_RIGHT_STACK_POSITIONS = {
           0.19, // Getting 1
           0.17, // Getting 2
           0.15, // Getting 3
           0.11, // Getting 4
           // Use WRIST_DOWN to get 5
   };

   public static double FEED_CLOSED = 0.69; // Nice
   public static double FEED_OPEN = 0.85;

   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
         this.intakeWristLeft = HardwareCreator.createServo(hardwareMap, "intakeWristLeft");
         this.intakeWristRight = HardwareCreator.createServo(hardwareMap, "intakeWristRight");
         this.intakeFeed = HardwareCreator.createServo(hardwareMap, "intakeFeed");
         this.beamBreak = new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, "intakeBeam"));
   }

   public void initialize() {
      this.intakeWristLeft.setPosition(WRIST_LEFT_STORED);
      this.intakeWristRight.setPosition(WRIST_RIGHT_STORED);
      this.intakeFeed.setPosition(FEED_CLOSED);
   }

   public Action intakeOn() {
      return intakeMotor.setTargetVelocityAction(INTAKE_SPEED);
   }

   public Action intakeReverse() {
      return intakeMotor.setTargetVelocityAction(INTAKE_REVERSE_SPEED);
   }

   public boolean isIntaking() {
      return intakeMotor.getTargetVelocity() == INTAKE_SPEED;
   }
   public boolean isReversing() {
      return intakeMotor.getTargetVelocity() == INTAKE_REVERSE_SPEED;
   }

   public Action intakeOff() {
      intakeMotor.resetIntegralGain();
      return intakeMotor.setTargetVelocityAction(0);
   }

   boolean beamBreakPrev = false;
   public int pixelCount = 0;
   public void update() {
      intakeMotor.update();
      if (beamBreak.isBeamBroken() && !beamBreakPrev) {
         beamBreakPrev = true;
         pixelCount++;
      }
      if (beamBreakPrev && !beamBreak.isBeamBroken()) {
         beamBreakPrev = false;
      }
   }

   public Action wristStored() {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_STORED), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_STORED));
   }
   public boolean wristIsStored() {
      return Math.abs(intakeWristLeft.getPosition() - WRIST_LEFT_STORED) < EPSILON;
   }

   public Action wristDown() {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_DOWN), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_DOWN));
   }

   public Action wristStack(int numberIntaked) {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_STACK_POSITIONS[numberIntaked]), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_STACK_POSITIONS[numberIntaked]));
   }

   private void wristStackInstant(int numberIntaked) {
      intakeWristLeft.setPosition(WRIST_LEFT_STACK_POSITIONS[numberIntaked]);
      intakeWristRight.setPosition(WRIST_RIGHT_STACK_POSITIONS[numberIntaked]);
   }

   public Action feedClosed() {
      return new ActionUtil.ServoPositionAction(intakeFeed, FEED_CLOSED);
   }

   public Action feedOpen() {
      return new ActionUtil.ServoPositionAction(intakeFeed, FEED_OPEN);
   }

   int numIntaked = 0;
   public Action intakeCount(boolean start) {
      if (start) {
         numIntaked = 0;
      }
      return new SequentialAction(
              intakeOn(),
              new IntakeCountAction(),
              intakeOff()
      );
   }
   private class IntakeCountAction implements Action {
      private long waitUntil;
      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {
         if (pixelCount < 2 && System.currentTimeMillis() >= waitUntil) {
            wristStackInstant(numIntaked);
            numIntaked++;
            this.waitUntil = System.currentTimeMillis() + 500;
         }
         return pixelCount != 2;
      }
   }
}
