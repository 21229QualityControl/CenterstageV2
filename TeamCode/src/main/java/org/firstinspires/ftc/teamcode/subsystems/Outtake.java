package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.DualMotorWithPID;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.007, 0.002, 0.0002);
   public static int OUTTAKE_TELEOP = 0; // Changes throughout teleop
   public static int OUTTAKE_BARELY_RAISED = 7; // Raises just enough to grab 1 pixel
   public static int LAYER_HEIGHT = 200; // Height of a layer of pixels for the slide, used for teleop
   public static int OUTTAKE_CLOSE = 525; // For close side auto
   public static int OUTTAKE_PARTNER = 650; // For far side auto, start of TeleOp
   public static int OUTTAKE_CYCLE = 1100; // For cycling during auto
   public static int OUTTAKE_CYCLE_HIGH = 1400; // For cycling during auto
   public static int OUTTAKE_HANG_EXTEND = 1550;
   public static int OUTTAKE_HANG_RETRACT = 800;

   public static double CLAW_OPEN = 0.6;
   public static double CLAW_CLOSED = 0.27;
   public static double CLAW_HALF_OPEN = 0.42;
   public static double CLAW_SINGLE_CLOSED = 0.21;

   public static double ARM_LEFT_STORED = 0.5;
   public static double ARM_LEFT_SCORING = 0.77;
   public static double ARM_RIGHT_STORED = 0.68;
   public static double ARM_RIGHT_SCORING = 0.42;

   public static double WRIST_VERTICAL = 0.43;
   public static double WRIST_VERTICAL_FLIPPED = 0.98;
   public static double WRIST_MOSAIC_LEFT = 0.51;
   public static double WRIST_MOSAIC_LEFT_FLIP = 1;
   public static double WRIST_MOSAIC_RIGHT = 0.34;
   public static double WRIST_MOSAIC_RIGHT_FLIP = 0.89;
   public static double WRIST_LEFT = 0.71;
   public static double WRIST_RIGHT = 0.145;

   public static double MOSAIC_FIXING = 0.85;
   public static double MOSAIC_STORED = 0.18;

   final DualMotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo claw;
   final Servo wrist;
   final Servo armLeft;
   final Servo armRight;
   final Servo mosaic;

   public double outtakeFF(double target, double measured, double vel) {
      if ((target + slide.internalOffset) == 0 && (measured + slide.internalOffset) > 5 && Math.abs(this.slide.getVelocity()) < 5) {
         return -0.5;
      }
      return 0;
   }

   public Outtake(HardwareMap hardwareMap) {
      this.slide = new DualMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlideWithEncoder"), HardwareCreator.createMotor(hardwareMap, "secondaryOuttakeSlide"), outtakePID, this::outtakeFF);
      this.slide.setMaxPower(1.0);
      this.claw = HardwareCreator.createServo(hardwareMap, "outtakeClaw");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
      this.armLeft = HardwareCreator.createServo(hardwareMap, "outtakeArmLeft");
      this.armRight = HardwareCreator.createServo(hardwareMap, "outtakeArmRight");
      this.mosaic = HardwareCreator.createServo(hardwareMap, "mosaic");
   }

   public void initialize(boolean teleop) {
      this.slide.resetIntegralGain();
      OUTTAKE_TELEOP = OUTTAKE_PARTNER;
      this.claw.setPosition(CLAW_OPEN);
      this.wrist.setPosition(WRIST_VERTICAL);
      this.mosaic.setPosition(MOSAIC_STORED);
      if (!Memory.FINISHED_AUTO && teleop) {
         this.slide.setTargetPosition(OUTTAKE_PARTNER);
         this.armLeft.setPosition(ARM_LEFT_SCORING);
//         this.armRight.setPosition(ARM_RIGHT_SCORING);
         this.claw.setPosition(CLAW_OPEN);
         Memory.FINISHED_AUTO = true;
      } else {
         this.slide.setTargetPosition(0);
         this.armLeft.setPosition(ARM_LEFT_STORED);
//         this.armRight.setPosition(ARM_RIGHT_STORED);
      }
   }

   public void prepInitializeSlides() {
      this.wrist.setPosition(WRIST_VERTICAL);
      this.armLeft.setPosition(ARM_LEFT_STORED);
      this.armRight.setPosition(ARM_RIGHT_STORED);
      this.slide.setPower(-0.6);
   }

   public boolean initializeSlides() {
      if ((int)this.slide.getVelocity() == 0) {
         this.slide.setPower(0);
         this.slide.setCurrentPosition(0);
         this.slide.setTargetPosition(0);
         return false;
      }
      return true;
   }

   public void update() {
      Log.d("POW", "update");
      if (slidePIDEnabled) {
         slide.update();
      }
   }

   public void setSlidePower(double power) {
      slide.setPower(power);
   }
   public void resetSlideOffset() {
      this.slide.setCurrentPosition(0);
      this.slide.setTargetPosition(0);
   }
   public Action lockPosition() {
      int pos = this.slide.getCurrentPosition();
      return this.slide.setTargetPositionAction(pos);
   }
   public Action increaseSlideLayer(int cnt) {
      OUTTAKE_TELEOP += LAYER_HEIGHT * cnt;
      return this.slide.setTargetPositionAction(OUTTAKE_TELEOP);
   }

   public Action extendOuttakeBarelyOut() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_BARELY_RAISED);
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
   public Action extendOuttakeCycleHighBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_CYCLE_HIGH);
   }

   public Action extendOuttakeTeleopBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_TELEOP);
   }
   public Action extendOuttakeHangBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_HANG_EXTEND);
   }
   public Action retractOuttakeHang() {
      return this.slide.setTargetPositionAction(OUTTAKE_HANG_RETRACT);
   }
   public boolean isSlideRetracted() {
      return this.slide.getTargetPosition() == 0;
   }
   public boolean isSlideHanging() {
      return this.slide.getTargetPosition() == OUTTAKE_HANG_EXTEND;
   }

   public Action retractOuttakeBlocking() {
      return this.slide.setTargetPositionActionBlocking(0);
   }
   public Action retractOuttake() {
      return this.slide.setTargetPositionAction(0);
   }

   public Action clawOpen() { return new ActionUtil.ServoPositionAction(claw, CLAW_OPEN); }
   public Action clawHalfOpen() { return new ActionUtil.ServoPositionAction(claw, CLAW_HALF_OPEN); }

   public Action clawClosed() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_CLOSED);
   }

   public Action clawSingleClosed() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_SINGLE_CLOSED);
   }

   public Action wristMosaic(boolean left) {
      if (left) {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_LEFT);
      } else {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_RIGHT);
      }
   }

   public Action wristMosaicFlip(boolean left) {
      if (left) {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_LEFT_FLIP);
      } else {
         return new ActionUtil.ServoPositionAction(wrist, WRIST_MOSAIC_RIGHT_FLIP);
      }
   }

   public boolean isWristMosaic(boolean left) {
      return (Math.abs(wrist.getPosition() - (left ? WRIST_MOSAIC_LEFT : WRIST_MOSAIC_RIGHT)) < EPSILON) || (Math.abs(wrist.getPosition() - (left ? WRIST_MOSAIC_RIGHT_FLIP : WRIST_MOSAIC_LEFT_FLIP)) < EPSILON);
   }

   public boolean isWristVertical() {
      return (Math.abs(wrist.getPosition() - WRIST_VERTICAL) < EPSILON) || (Math.abs(wrist.getPosition() - WRIST_VERTICAL_FLIPPED) < EPSILON);
   }

   public boolean isWristSideways() {
      return (Math.abs(wrist.getPosition() - WRIST_LEFT) < EPSILON) || (Math.abs(wrist.getPosition() - WRIST_RIGHT) < EPSILON);
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

   public Action wristVerticalFlip() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_VERTICAL_FLIPPED);
   }

   public Action armStored() {
      return new ParallelAction(
              new ActionUtil.ServoPositionAction(armLeft, ARM_LEFT_STORED)
//              new ActionUtil.ServoPositionAction(armRight, ARM_RIGHT_STORED)
      );
   }

   public Action armScoring() {
      return new ParallelAction(
              new ActionUtil.ServoPositionAction(armLeft, ARM_LEFT_SCORING)
//              new ActionUtil.ServoPositionAction(armRight, ARM_RIGHT_SCORING)
      );
   }
   public boolean isArmScoring() {
      return Math.abs(armLeft.getPosition() - ARM_LEFT_SCORING) < EPSILON;
   }

   public Action mosaicFix() {
      return new ActionUtil.ServoPositionAction(mosaic, MOSAIC_FIXING);
   }

   public boolean isMosaicFixing() {
      return Math.abs(mosaic.getPosition() - MOSAIC_FIXING) < EPSILON;
   }

   public Action mosaicStored() {
      return new ActionUtil.ServoPositionAction(mosaic, MOSAIC_STORED);
   }
}
