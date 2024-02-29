package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

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
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.007, 0.0015, 0.0002);
   public static int OUTTAKE_TELEOP = 0; // Changes throughout teleop
   public static int LAYER_HEIGHT = 100; // Height of a layer of pixels for the slide, used for teleop
   public static int OUTTAKE_CLOSE = 200; // For close side auto
   public static int OUTTAKE_PARTNER = 275; // For far side auto, start of TeleOp
   public static int OUTTAKE_CYCLE = 375; // For cycling during auto
   public static int OUTTAKE_HANG = 575;

   public static double CLAW_OPEN = 0.74;
   public static double CLAW_CLOSED = 0.54;
   public static double CLAW_PRELOAD_CLOSED = 0.47;


   public static double ARM_LEFT_STORED = 0.47;
   public static double ARM_LEFT_SCORING = 0.725;
   public static double ARM_RIGHT_STORED; // the arm right servo is weird so I'm not using it for now
   public static double ARM_RIGHT_SCORING;


   public static double WRIST_VERTICAL = 0.545;
   public static double WRIST_MOSAIC_LEFT = 0;
   public static double WRIST_MOSAIC_RIGHT = 0.42;
   public static double WRIST_LEFT = 0.88;
   public static double WRIST_RIGHT = 0.2;

   public double mosaicPosition;
   final DualMotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo claw;
   final Servo wrist;
   final Servo armLeft;
   final Servo armRight;

   public Outtake(HardwareMap hardwareMap) {
      if (Memory.outtakeSlide != null) { // Preserve motor zero position
         this.slide = Memory.outtakeSlide;
      } else {
         this.slide = new DualMotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlideWithEncoder"), HardwareCreator.createMotor(hardwareMap, "secondaryOuttakeSlide"), outtakePID);
         Memory.outtakeSlide = this.slide;
      }
      this.slide.setMaxPower(1.0);
      this.claw = HardwareCreator.createServo(hardwareMap, "outtakeClaw");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
      this.armLeft = HardwareCreator.createServo(hardwareMap, "outtakeArmLeft");
      this.armRight = HardwareCreator.createServo(hardwareMap, "outtakeArmRight");
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
         this.armLeft.setPosition(ARM_LEFT_SCORING);
         this.armRight.setPosition(ARM_RIGHT_SCORING);
         this.claw.setPosition(CLAW_OPEN);
         Memory.FINISHED_AUTO = true;
      } else {
         this.slide.setTargetPosition(0);
         this.armLeft.setPosition(ARM_LEFT_STORED);
         this.armRight.setPosition(ARM_RIGHT_STORED);
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
   public Action extendOuttakeHangBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_HANG);
   }
   public boolean isSlideRetracted() {
      return this.slide.getTargetPosition() == 0;
   }

   public Action retractOuttakeBlocking() {
      return this.slide.setTargetPositionActionBlocking(0);
   }
   public Action retractOuttake() {
      return this.slide.setTargetPositionAction(0);
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
      return new ParallelAction(
              new ActionUtil.ServoPositionAction(armLeft, ARM_LEFT_STORED),
              new ActionUtil.ServoPositionAction(armRight, ARM_RIGHT_STORED)
      );
   }

   public Action armScoring() {
      return new ParallelAction(
              new ActionUtil.ServoPositionAction(armLeft, ARM_LEFT_SCORING),
              new ActionUtil.ServoPositionAction(armRight, ARM_RIGHT_SCORING)
      );
   }
   public boolean isArmScoring() {
      return Math.abs(armLeft.getPosition() - ARM_LEFT_SCORING) < EPSILON;
   }
}
