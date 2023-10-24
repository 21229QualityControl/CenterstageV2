package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public class Intake {
   public static double INTAKE_POWER = 0.7;
   public static double AUTO_LATCH_OPEN = 0.92;
   public static double AUTO_LATCH_DOWN = 0.65;

   final DcMotorEx intakeMotor;
   final Servo autoLatch;

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intakeMotor");
         this.autoLatch = HardwareCreator.createServo(hardwareMap, "autoLatch");
   }

   public enum IntakeState {
      Off,
      Reversing,
      On
   }
   public IntakeState intakeState;

   public void initializeAuto() {
      autoLatch.setPosition(AUTO_LATCH_DOWN);
   }

   public void initializeTeleop() {
      autoLatch.setPosition(AUTO_LATCH_OPEN);
   }

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

   public Action intakeOn() {
      return new SequentialAction(
              new ActionUtil.DcMotorExPowerAction(intakeMotor, INTAKE_POWER),
              new IntakeStateAction(IntakeState.On)
      );
   }

   public Action intakeReverse() {
      return new SequentialAction(
              new ActionUtil.DcMotorExPowerAction(intakeMotor, -INTAKE_POWER),
              new IntakeStateAction(IntakeState.Reversing)
      );
   }

   public Action intakeOff() {
      return new SequentialAction(
              new ActionUtil.DcMotorExPowerAction(intakeMotor, 0.0),
              new IntakeStateAction(IntakeState.Off)
      );
   }

   public Action autoLatchOpen() {
      return new ActionUtil.ServoPositionAction(autoLatch, AUTO_LATCH_OPEN);
   }

   public Action autoLatchDown() {
      return new ActionUtil.ServoPositionAction(autoLatch, AUTO_LATCH_DOWN);
   }
}
