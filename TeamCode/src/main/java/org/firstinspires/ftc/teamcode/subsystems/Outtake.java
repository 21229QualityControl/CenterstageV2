package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.03, 0, 0.0004);
   public static int OUTTAKE_EXTENDED = 1521;
   public static double LATCH_SCORING = 0.976;
   public static double LATCH_OPEN = 0.823;
   public static double LATCH_CLOSED = 1;
   public static double WRIST_STORED = 0.27;
   public static double WRIST_SCORING = 0.558;
   final MotorWithPID slide;
   final Servo latch;
   final Servo wrist;

   public Outtake(HardwareMap hardwareMap) {
      this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide"), outtakePID);
      this.slide.setMaxPower(1.0);
      this.slide.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
      this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
   }

   public void prepTeleop() {
      this.slide.getMotor().setPower(-0.3);
   }

   public void finishPrepTeleop() {
      this.slide.getMotor().setPower(0);
   }

   public void initializeTeleop() {
      this.slide.setTargetPosition(0);
      this.wrist.setPosition(WRIST_STORED);
      this.latch.setPosition(LATCH_OPEN);
   }

   public void update() {
      slide.update();
   }

   public Action extendOuttake() {
      return this.slide.setTargetPositionAction(OUTTAKE_EXTENDED);
   }
   public Action extendOuttakeBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_EXTENDED);
   }
   public Action retractOuttake() {
      return this.slide.setTargetPositionAction(0);
   }

   public Action latchOpen() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_OPEN);
   }

   public Action latchScoring() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_SCORING);
   }

   public Action latchClosed() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED);
   }

   public Action wristStored() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_STORED);
   }

   public Action wristScoring() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_SCORING);
   }
}
