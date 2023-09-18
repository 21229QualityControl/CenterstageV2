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
   final DcMotorEx intakeMotor;

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intakeMotor");
   }

   public Action intakeOn() {
      return new ActionUtil.DcMotorExPowerAction(intakeMotor, 1.0);
   }

   public Action intakeOff() {
      return new ActionUtil.DcMotorExPowerAction(intakeMotor, 0.0);
   }
}
