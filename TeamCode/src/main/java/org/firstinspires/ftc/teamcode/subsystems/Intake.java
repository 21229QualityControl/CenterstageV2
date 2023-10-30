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

   final DcMotorEx intakeMotor;

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intakeMotor");
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
}
