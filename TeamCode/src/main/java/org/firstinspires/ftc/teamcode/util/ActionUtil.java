package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionUtil {
   public static class DcMotorExPowerAction implements Action {
      double power;
      DcMotorEx motor;

      public DcMotorExPowerAction(DcMotorEx motor, double power) {
         this.power = power;
         this.motor = motor;
      }
      @Override
      public void preview(Canvas c) {
         motor.setPower(power);
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         return false;
      }
   }

   public static class ServoPositionAction implements Action {
      double position;
      Servo servo;

      public ServoPositionAction(Servo servo, double position) {
         this.servo = servo;
         this.position = position;
      }
      @Override
      public void preview(Canvas c) {
         servo.setPosition(position);
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         return false;
      }
   }
}
