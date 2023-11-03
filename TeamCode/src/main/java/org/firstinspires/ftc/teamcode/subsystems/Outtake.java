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
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.01, 0, 0.0004);
   public static int OUTTAKE_MID = 1200;
   public static int OUTTAKE_LOW = 400;
   public static double LATCH_SCORING = 0.855;
   public static double LATCH_OPEN = 0.8;
   public static double LATCH_CLOSED = 0.88;
   public static double WRIST_STORED = 0.66;
   public static double WRIST_SCORING = 0.41;
   final MotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo latch;
   final Servo wrist;

   public Outtake(HardwareMap hardwareMap) {
      if (Memory.outtakeSlide != null) { // Preserve motor zero position
         this.slide = Memory.outtakeSlide;
      } else {
         this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide"), outtakePID);
         Memory.outtakeSlide = this.slide;
      }
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

   public void initialize() {
      this.slide.setTargetPosition(0);
      this.wrist.setPosition(WRIST_STORED);
      this.latch.setPosition(LATCH_CLOSED);
   }

   public void resetMotors() {
      this.slide.setCurrentPosition(0);
   }

   public void update() {
      if (slidePIDEnabled) {
         slide.update();
      }
   }

   public Action extendOuttakeMid() {
      return this.slide.setTargetPositionAction(OUTTAKE_MID);
   }

   public void setSlidePower(double power) {
      slide.getMotor().setPower(power);
   }
   public void lockPosition() {
      this.slide.setTargetPosition(this.slide.getCurrentPosition());
   }

   public Action extendOuttakeMidBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_MID);
   }
   public Action extendOuttakeLowBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_LOW);
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
