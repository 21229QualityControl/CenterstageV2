package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 1000; // Max speed is 2400

   final MotorWithVelocityPID intakeMotor;
   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) { 
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
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
   }
}
