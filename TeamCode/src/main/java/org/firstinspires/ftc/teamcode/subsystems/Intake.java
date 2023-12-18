package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
   public static int INTAKE_SPEED = 1000; // Max speed is 2400

   final MotorWithVelocityPID intakeMotor;
   final Servo leftStack;
   final Servo rightStack;
   final BeamBreakSensor beamBreak;
   boolean beamBroken;

   public int pixelCount = 0;

   public static double LEFT_STACK_OPEN = 0.6;
   public static double LEFT_STACK_CLOSED = 0.1;
   public static double RIGHT_STACK_OPEN = 0.03;
   public static double RIGHT_STACK_CLOSED = 0.47;
   public static double RIGHT_STACK_HALF = (RIGHT_STACK_OPEN + RIGHT_STACK_CLOSED) / 2;
   public static double LEFT_STACK_HALF = (LEFT_STACK_OPEN + RIGHT_STACK_OPEN) / 2;

   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) { 
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
         this.leftStack = HardwareCreator.createServo(hardwareMap, "leftStack");
         this.rightStack = HardwareCreator.createServo(hardwareMap, "rightStack");
         this.beamBreak = new BeamBreakSensor(hardwareMap, "intakeBeam");
   }

   public void initialize() {
      this.leftStack.setPosition(LEFT_STACK_OPEN);
      this.rightStack.setPosition(RIGHT_STACK_OPEN);
   }

   public enum IntakeState {
      Off,
      Reversing,
      On
   }
   public IntakeState intakeState;

   public class IntakeStateAction implements Action {
      IntakeState newState;

      public IntakeStateAction(IntakeState newState) {
         this.newState = newState;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         intakeState = newState;
         return false;
      }
   }
   public class IntakePixelCountAction implements Action {
      int newPixelCount;

      public IntakePixelCountAction(int newPixelCount) {
         this.newPixelCount = newPixelCount;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         pixelCount = newPixelCount;
         return false;
      }
   }

   public Action intakeOn() {
      return new SequentialAction(
              intakeMotor.setTargetVelocityAction(INTAKE_SPEED),
              //new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), INTAKE_SPEED / 1000.0),
              new IntakeStateAction(IntakeState.On)
      );
   }

   public Action intakeReverse() {
      return new SequentialAction(
              intakeMotor.setTargetVelocityAction(-INTAKE_SPEED),
              //new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), -INTAKE_SPEED / 1000.0),
              new IntakeStateAction(IntakeState.Reversing)
      );
   }

   public Action intakeOff() {
      intakeMotor.resetIntegralGain();
      return new SequentialAction(
              intakeMotor.setTargetVelocityAction(0),
              //new ActionUtil.DcMotorExPowerAction(intakeMotor.getMotor(), 0),
              new IntakeStateAction(IntakeState.Off)
      );
   }

   public void update() {
      intakeMotor.update();
      if (!beamBreak.isBeamBroken() && beamBroken) {
         pixelCount++;
         if (pixelCount == 2 && intakeState == IntakeState.On) {
            //Actions.runBlocking(intakeOff()); // TODO: Increase beam break reliability
         }
      }
      beamBroken = beamBreak.isBeamBroken();
   }

   public Action stackOpen() {
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(leftStack, LEFT_STACK_OPEN),
              new ActionUtil.ServoPositionAction(rightStack, RIGHT_STACK_OPEN)
      );
   }

   public Action stackHalf() {
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(leftStack, LEFT_STACK_HALF),
              new ActionUtil.ServoPositionAction(rightStack, RIGHT_STACK_HALF)
      );
   }

   public Action stackClosed() {
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(leftStack, LEFT_STACK_CLOSED),
              new ActionUtil.ServoPositionAction(rightStack, RIGHT_STACK_CLOSED)
      );
   }

   public Action setPixelCount(int newPixelCount) {
      return new IntakePixelCountAction(newPixelCount);
   }
}
