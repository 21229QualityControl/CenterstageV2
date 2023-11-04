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
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 700;

   final MotorWithVelocityPID intakeMotor;
   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.1, 0, 0);

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
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
              intakeMotor.setTargetVelocityAction(INTAKE_SPEED),
              new IntakeStateAction(IntakeState.On)
      );
   }

   public Action intakeReverse() {
      return new SequentialAction(
              intakeMotor.setTargetVelocityAction(-INTAKE_SPEED),
              new IntakeStateAction(IntakeState.Reversing)
      );
   }

   public Action intakeOff() {
      return new SequentialAction(
              intakeMotor.setTargetVelocityAction(0),
              new IntakeStateAction(IntakeState.Off)
      );
   }
}
