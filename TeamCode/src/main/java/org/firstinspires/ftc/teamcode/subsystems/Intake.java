package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 1000; // Max speed is 2400
   public static int INTAKE_SLOW = 2400; //previously 600
   public static int INTAKE_REVERSE_SPEED = -2400;

   final MotorWithVelocityPID intakeMotor;
   final Servo intakeWristLeft;
   final Servo intakeWristRight;
   final Servo intakeFeed;

   final BeamBreakSensor beamBreakFront;
   final BeamBreakSensor beamBreakBack;

   final DistanceSensor distanceSensor1;
   private DistanceSensor distanceSensor2;
   final DistanceSensor distanceLeft;
   final DistanceSensor distanceRight;

   public static double WRIST_LEFT_STORED = 0.17;
   public static double WRIST_LEFT_DOWN = 0.43;
   public static double WRIST_LEFT_PRELOAD = 0.45;
   public static double[] WRIST_LEFT_STACK_POSITIONS = {
           0.34, // Getting 1
           0.36, // Getting 2
           0.37, // Getting 3
           0.40, // Getting 4
           WRIST_LEFT_DOWN,
   };

   public static double WRIST_RIGHT_STORED = 0.34;
   public static double WRIST_RIGHT_DOWN = 0.1;
   public static double WRIST_RIGHT_PRELOAD = 0.07;
   public static double[] WRIST_RIGHT_STACK_POSITIONS = {
           0.18, // Getting 1
           0.16, // Getting 2
           0.14, // Getting 3
           0.12, // Getting 4
           WRIST_RIGHT_DOWN,
   };

   public static double FEED_CLOSED = 0.69; // Nice
   public static double FEED_OPEN = 0.85;

   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
         this.intakeMotor.getMotor().setCurrentAlert(5.5, CurrentUnit.AMPS);
         this.intakeWristLeft = HardwareCreator.createServo(hardwareMap, "intakeWristLeft");
         this.intakeWristRight = HardwareCreator.createServo(hardwareMap, "intakeWristRight");
         this.intakeFeed = HardwareCreator.createServo(hardwareMap, "intakeFeed");
         this.beamBreakFront = new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, "intakeBeamFront"));
         this.beamBreakBack = new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, "intakeBeamBack"));

         this.distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor");
         this.distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
         this.distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
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
   public boolean isIntakeOverCurrent() {
      return intakeMotor.getMotor().isOverCurrent();
   }
   public Action intakeSlow() {
      return intakeMotor.setTargetVelocityAction(INTAKE_SLOW);
   }

   public Action intakeReverse() {
      return intakeMotor.setTargetVelocityAction(INTAKE_REVERSE_SPEED);
   }

   public void intakeReverseInstant() {
      intakeMotor.setTargetVelocity(INTAKE_REVERSE_SPEED);
   }

   public boolean isIntaking() {
      return intakeMotor.getTargetVelocity() == INTAKE_SPEED;
      // isIntaking seems to return false when the intake is reversing.
   }
   public boolean isReversing() {
      return intakeMotor.getTargetVelocity() == INTAKE_REVERSE_SPEED;
   }

   public Action intakeOff() {
      intakeMotor.resetIntegralGain();
      return intakeMotor.setTargetVelocityAction(0);
   }
   public void update() {
      intakeMotor.update();
   }

   public int pixelCount() {
      if (beamBreakBack.isBeamBroken()) {
         if (beamBreakFront.isBeamBroken()) {
            return 2;
         }
         return 1;
      }
      return 0;
   }

   public Action wristStored() {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_STORED), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_STORED));
   }
   public void wristStoredInstant() {
      intakeWristLeft.setPosition(WRIST_LEFT_STORED);
      intakeWristRight.setPosition(WRIST_RIGHT_STORED);
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

   public double sideDistance(boolean right) {
      if (right) {
         return distanceRight.getDistance(DistanceUnit.INCH);
      } else {
         return distanceLeft.getDistance(DistanceUnit.INCH);
      }
   }

   public int numIntaked = 0;
   public Action prepIntakeCount(boolean start, boolean one) {
      if (start) {
         numIntaked = -1;
      }
      if (one) {
         numIntaked++;
      } else {
         numIntaked += 2; // Intake 2 at once
      }
      if (numIntaked >= WRIST_LEFT_STACK_POSITIONS.length) {
         numIntaked = WRIST_LEFT_STACK_POSITIONS.length-1;
      }
      return new SequentialAction(
              intakeOn(),
              wristStack(numIntaked)
      );
   }
   public Action intakeCount(boolean fast) {
      return new SequentialAction(
              new IntakeCountAction(fast),
              intakeReverse()
      );
   }
   private class IntakeCountAction implements Action {
      private long waitUntil;
      private long finalTime;
      private boolean done;
      private boolean jammed;

      public IntakeCountAction(boolean fast) {
         this.waitUntil = System.currentTimeMillis() + 300;
         this.finalTime = System.currentTimeMillis() + (fast ? 3000 : 6000);
      }

      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {
         if (done) { // Make it wait 0.2s after it has intaked 2
            return System.currentTimeMillis() < waitUntil;
         }
         if (System.currentTimeMillis() >= finalTime) {
            wristStoredInstant();
         }

         if (jammed) {
            if (!intakeMotor.getMotor().isOverCurrent()) {
               this.jammed = false;
               this.waitUntil = System.currentTimeMillis() + 600;
               wristStackInstant(numIntaked);
               return System.currentTimeMillis() < finalTime;
            }
            if (System.currentTimeMillis() >= waitUntil) {
               intakeReverseInstant();
               return System.currentTimeMillis() < finalTime;
            }
         }
         if (intakeMotor.getMotor().isOverCurrent()) {
            this.jammed = true;
            this.waitUntil = System.currentTimeMillis() + 200;
            wristStoredInstant();
            return System.currentTimeMillis() < finalTime;
         }

         if (pixelCount() < 2 && System.currentTimeMillis() >= waitUntil) {
            wristStackInstant(numIntaked);
            numIntaked++;
            if (numIntaked >= WRIST_LEFT_STACK_POSITIONS.length) {
               numIntaked = WRIST_LEFT_STACK_POSITIONS.length-1;
            }
            this.waitUntil = System.currentTimeMillis() + 600;
         }
         if (pixelCount() >= 2) {
            done = true;
            this.waitUntil = System.currentTimeMillis() + 200;
            wristStoredInstant();
            return true;
         }
         return System.currentTimeMillis() < finalTime;
      }
   }
}
