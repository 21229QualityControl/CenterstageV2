package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.DualMotorWithPID;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.01, 0.0015, 0.0004);
   public static int OUTTAKE_TELEOP = 0; // Changes throughout teleop
   public static int LAYER_HEIGHT = 100; // Height of a layer of pixels for the slide, used for teleop
   public static int OUTTAKE_CLOSE = 200; // For close side auto
   public static int OUTTAKE_PARTNER = 275; // For far side auto, start of TeleOp
   public static int OUTTAKE_CYCLE = 375; // For cycling during auto

   public static double CLAW_OPEN = 0.74;
   public static double CLAW_CLOSED = 0.5;
   public static double CLAW_PRELOAD_CLOSED = 0.3;
   public static double ARM_STORED = 0.651;
   public static double ARM_SCORING = 0;
   public static double WRIST_VERTICAL = 0;
   public static double WRIST_MOSAIC_LEFT = 0;
   public static double WRIST_MOSAIC_RIGHT = 0;
   public static double WRIST_LEFT = 0;
   public static double WRIST_RIGHT = 0;

   public double mosaicPosition;
   final DualMotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo claw;
   final Servo wrist;
   final Servo arm;

   public Outtake(HardwareMap hardwareMap) {
      if (Memory.outtakeSlide != null) { // Preserve motor zero position
         this.slide = Memory.outtakeSlide;
      } else {
         this.slide = new DualMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlideWithEncoder"), HardwareCreator.createMotor(hardwareMap, "secondOuttakeSlide"), outtakePID);
         Memory.outtakeSlide = this.slide;
      }
      this.slide.setMaxPower(1.0);
      this.claw = HardwareCreator.createServo(hardwareMap, "outtakeClaw");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
      this.arm = HardwareCreator.createServo(hardwareMap, "outtakeArm");
   }

   public void prepTeleop() {
      this.slide.setPower(-0.3);
   }

   public void finishPrepTeleop() {
      this.slide.setPower(0);
   }

   public void initialize(boolean teleop) {
      this.slide.resetIntegralGain();
      OUTTAKE_TELEOP = OUTTAKE_PARTNER;
      this.claw.setPosition(CLAW_OPEN);
      this.wrist.setPosition(WRIST_VERTICAL);
      if (!Memory.FINISHED_AUTO && teleop) {
         this.slide.setTargetPosition(OUTTAKE_PARTNER);
         this.arm.setPosition(ARM_SCORING);
         this.claw.setPosition(CLAW_OPEN);
         Memory.FINISHED_AUTO = true;
      } else {
         this.slide.setTargetPosition(0);
         this.arm.setPosition(ARM_STORED);
      }
   }

   public void resetMotors() {
      this.slide.setCurrentPosition(0);
   }

   public void update() {
      if (slidePIDEnabled) {
         slide.update();
      }
   }

   public void setSlidePower(double power) {
      slide.setPower(power);
   }
   public Action lockPosition() {
      int pos = this.slide.getCurrentPosition();
      return this.slide.setTargetPositionAction(pos);
   }
   public Action increaseSlideLayer(int cnt) {
      OUTTAKE_TELEOP += LAYER_HEIGHT * cnt;
      return this.slide.setTargetPositionAction(OUTTAKE_TELEOP);
   }

   public Action extendOuttakeCloseBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_CLOSE);
   }

   public Action extendOuttakePartnerBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_PARTNER);
   }

   public Action extendOuttakeCycleBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_CYCLE);
   }

   public Action extendOuttakeTeleopBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_TELEOP);
   }

   public Action retractOuttakeBlocking() {
      return this.slide.setTargetPositionActionBlocking(0);
   }

   public Action clawOpen() { return new ActionUtil.ServoPositionAction(claw, CLAW_OPEN); }

   public Action clawClosed() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_CLOSED);
   }

   public Action clawSingleClosed() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_PRELOAD_CLOSED);
   }

   public Action wristMosaic(boolean left) {
      if (left) {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_LEFT);
      } else {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_RIGHT);
      }
   }

   public boolean isWristMosaic() {
      return Math.abs(wrist.getPosition() - WRIST_MOSAIC_LEFT) < EPSILON || Math.abs(wrist.getPosition() - WRIST_MOSAIC_RIGHT) < EPSILON;
   }

   public Action wristSideways(boolean left) {
      if (left) {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_LEFT);
      } else {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_RIGHT);
      }
   }

   public Action wristVertical() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_VERTICAL);
   }

   public Action armStored() {
      return new ActionUtil.ServoPositionAction(arm, ARM_STORED);
   }

   public Action armScoring() {
      return new ActionUtil.ServoPositionAction(arm, ARM_SCORING);
   }
   public boolean isArmScoring() {
      return Math.abs(arm.getPosition() - ARM_SCORING) < EPSILON;
   }
}
