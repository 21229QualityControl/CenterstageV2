package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public class Intake {
   public static double INTAKE_POWER = 1.0;
   public static double AUTO_LATCH_OPEN = 0;
   public static double AUTO_LATCH_DOWN = 1;

   final DcMotorEx intakeMotor;
   final Servo autoLatch;

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = HardwareCreator.createMotor(hardwareMap, "intakeMotor");
         this.autoLatch = HardwareCreator.createServo(hardwareMap, "autoLatch");
   }

   public void initialize() {
      autoLatch.setPosition(AUTO_LATCH_DOWN);
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

   public Action autoLatchOpen() {
      return new ActionUtil.ServoPositionAction(autoLatch, AUTO_LATCH_OPEN);
   }

   public Action autoLatchDown() {
      return new ActionUtil.ServoPositionAction(autoLatch, AUTO_LATCH_DOWN);
   }
}
