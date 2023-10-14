package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public class Intake {
   public static double INTAKE_POWER = 0.5;

   final DcMotorEx intakeMotor;

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intakeMotor");
   }

   public boolean isIntakeOn() {
      return intakeMotor.getPower() >= INTAKE_POWER;
   }

   public Action intakeOn() {
      return new ActionUtil.DcMotorExPowerAction(intakeMotor, INTAKE_POWER);
   }

   public Action intakeOff() {
      return new ActionUtil.DcMotorExPowerAction(intakeMotor, 0.0);
   }
}
