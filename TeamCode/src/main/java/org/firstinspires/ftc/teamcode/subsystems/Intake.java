package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 600; // Max speed is 2400
   public static int INTAKE_REVERSE_SPEED = -2400;

   final MotorWithVelocityPID intakeMotor;
   final Servo intakeWristLeft;
   final Servo intakeWristRight;
   final Servo intakeFeed;

   final BeamBreakSensor beamBreak;
   private DistanceSensor distanceSensor1;
   private DistanceSensor distanceSensor2;

   public static double WRIST_LEFT_STORED = 0.17;
   public static double WRIST_LEFT_DOWN = 0.43;
   public static double WRIST_LEFT_PRELOAD = 0.45;
   public static double[] WRIST_LEFT_STACK_POSITIONS = {
           0.35, // Getting 1
           0.37, // Getting 2
           0.39, // Getting 3
           0.41, // Getting 4
           WRIST_LEFT_DOWN,
   };

   public static double WRIST_RIGHT_STORED = 0.34;
   public static double WRIST_RIGHT_DOWN = 0.09;
   public static double WRIST_RIGHT_PRELOAD = 0.07;
   public static double[] WRIST_RIGHT_STACK_POSITIONS = {
           0.17, // Getting 1
           0.15, // Getting 2
           0.13, // Getting 3
           0.11, // Getting 4
           WRIST_RIGHT_DOWN,
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
         this.distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor");
   }

   public void initialize() {
      this.intakeWristLeft.setPosition(WRIST_LEFT_STORED);
      this.intakeWristRight.setPosition(WRIST_RIGHT_STORED);
      this.intakeFeed.setPosition(FEED_CLOSED);
   }

   public boolean isIntakeNearWall() {
      return distanceSensor1.getDistance(DistanceUnit.INCH) < 36;
   }

   public boolean willIntakeHitWall() {
      return distanceSensor1.getDistance(DistanceUnit.MM) < 150;
   }

   public double getDistance() {
      return distanceSensor1.getDistance(DistanceUnit.MM);
   }

   public Action intakeOn() {
      return intakeMotor.setTargetVelocityAction(INTAKE_SPEED);
   }

   public Action intakeReverse() {
      Log.d("SETPOWER", "REVERSE");
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
      if (pixelCount == 1 && beamBreak.isBeamBroken()) {
         pixelCount = 2;
         return;
      }
      if (pixelCount > 0) {
         return;
      }

      if (beamBreak.isBeamBroken() && !beamBreakPrev) {
         beamBreakPrev = true;
      }
      if (beamBreakPrev && !beamBreak.isBeamBroken()) {
         beamBreakPrev = false;
         pixelCount++;
      }
   }

   public Action setPixelCount(int count) {
      return new ActionUtil.RunnableAction(() -> {
         pixelCount = count;
         return false;
      });
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
   public Action wristPreload() {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_PRELOAD), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_PRELOAD));
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

   public int numIntaked = 0;
   public Action prepIntakeCount(boolean start) {
      if (start) {
         numIntaked = 0;
      }
      numIntaked++; // Intake 2 at once
      if (numIntaked >= WRIST_LEFT_STACK_POSITIONS.length) {
         numIntaked = WRIST_LEFT_STACK_POSITIONS.length-1;
      }
      return new SequentialAction(
              intakeOn(),
              wristStack(numIntaked)
      );
   }
   public Action intakeCount() {
      return new SequentialAction(
              new IntakeCountAction(),
              intakeReverse(),
              wristStored()
      );
   }
   private class IntakeCountAction implements Action {
      private long waitUntil;

      public IntakeCountAction() {
         this.waitUntil = System.currentTimeMillis() + 1500;
      }

      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {
         if (pixelCount < 2 && System.currentTimeMillis() >= waitUntil) {
            wristStackInstant(numIntaked);
            numIntaked++;
            if (numIntaked >= WRIST_LEFT_STACK_POSITIONS.length) {
               numIntaked = WRIST_LEFT_STACK_POSITIONS.length-1;
            }
            this.waitUntil = System.currentTimeMillis() + 1500;
         }
         return pixelCount != 2;
      }
   }
}
