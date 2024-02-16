package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MagnetSwitchSensor;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.01, 0.0015, 0.0004);
   public static int OUTTAKE_TELEOP = 0;
   public static int LAYER_HEIGHT = 100; // Height of a layer of pixels for the slide, used for teleop
   public static int OUTTAKE_MIDLOW = 325;
   public static int OUTTAKE_MID = 600;
   public static int OUTTAKE_LOW = 250;
   public static double CLAW_OPEN = 0.74;
   public static double CLAW_CLOSED = 0.525;
   public static double WRIST_STORED = 0.651;
   public static double WRIST_SCORING = 0.9;
   public static double CLAW_WRIST_DEFAULT = 0.33;
   public static double CLAW_WRIST_SLANT_1 = 0.435;
   public static double CLAW_WRIST_SLANT_2 = 0.2;
   public static double CLAW_WRIST_HORIZONTAL_1 = 0.67;
   public static double CLAW_WRIST_HORIZONTAL_2 = 0.01;

   public static double[] clawWristPositions = {CLAW_WRIST_DEFAULT,
                   CLAW_WRIST_SLANT_1,
                   CLAW_WRIST_SLANT_2,
                   CLAW_WRIST_HORIZONTAL_1,
                   CLAW_WRIST_HORIZONTAL_2};

   public static double MOSAIC_ADJUSTING = 0.55;
   public static double MOSAIC_CLOSED = 0.075;
   public static boolean NEED_RESET = false;

   public double mosaicPosition;
   final MotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo claw;
   final Servo wrist;
   final MagnetSwitchSensor slideSensor;
   final Servo mosaic;
   final TouchSensor touchSensor;
   final Servo clawWrist;

   public Outtake(HardwareMap hardwareMap) {
      if (Memory.outtakeSlide != null) { // Preserve motor zero position
         this.slide = Memory.outtakeSlide;
      } else {
         this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide"), outtakePID);
         Memory.outtakeSlide = this.slide;
      }
      this.slide.setMaxPower(1.0);
      this.slide.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
      this.claw = HardwareCreator.createServo(hardwareMap, "outtakeClaw");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
      this.slideSensor = new MagnetSwitchSensor(hardwareMap, "outtakeMagnetSensor");
      this.mosaic = HardwareCreator.createServo(hardwareMap, "mosaic");
      this.touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
      this.clawWrist = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
   }
   // The object outtake can get the slide motor to be directly used in Manual Drive.
   public MotorWithPID getSlide(){
      return this.slide;
   }

   public void prepTeleop() {
      this.slide.getMotor().setPower(-0.3);
   }

   public void finishPrepTeleop() {
      this.slide.getMotor().setPower(0);
   }

   public void initialize(boolean teleop) {
      this.slide.resetIntegralGain();
      OUTTAKE_TELEOP = OUTTAKE_MIDLOW;
      this.mosaic.setPosition(MOSAIC_CLOSED);
      this.claw.setPosition(CLAW_OPEN);
      this.clawWrist.setPosition(CLAW_WRIST_DEFAULT);
      Log.d("BACKDROP_FINISHEDAUTO", String.valueOf(Memory.FINISHED_AUTO));
      if (!Memory.FINISHED_AUTO && teleop) {
         this.slide.setTargetPosition(OUTTAKE_MID);
         this.wrist.setPosition(WRIST_SCORING);
         Memory.FINISHED_AUTO = true;
      } else {
         this.slide.setTargetPosition(0);
         this.wrist.setPosition(WRIST_STORED);
         NEED_RESET = true;
      }
   }

   public void resetMotors() {
      this.slide.setCurrentPosition(0);
   }

   public void update() {
      if (slidePIDEnabled) {
         slide.update();
      }
      if (NEED_RESET && isSlideDown()) {
         slide.zeroMotorInternals();
         slide.resetIntegralGain();
      }
   }

   public Servo getClawWrist() {
      return clawWrist;
   }
   public void setSlidePower(double power) {
      slide.getMotor().setPower(power);
   }
   public Action lockPosition() {
      int pos = this.slide.getCurrentPosition();
      return this.slide.setTargetPositionAction(pos);
   }
   public Action increaseSlideLayer(int cnt) {
      OUTTAKE_TELEOP += LAYER_HEIGHT * cnt;
      return this.slide.setTargetPositionAction(OUTTAKE_TELEOP);
   }

   public Action extendOuttakeMidBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_MID);
   }

   public Action extendOuttakeMid() {
      return this.slide.setTargetPositionAction(OUTTAKE_MID);
   }

   public Action extendOuttakeMidLow() {
      return this.slide.setTargetPositionAction(OUTTAKE_MIDLOW);
   }

   public Action extendOuttakeTeleopBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_TELEOP);
   }
   public Action extendOuttakeLowBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_LOW);
   }
   public Action retractOuttake() {
      NEED_RESET = true;
      return this.slide.setTargetPositionAction(0);
   }

   public Action latchScoring() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_OPEN);
   }
   public boolean isLatchScoring() {
      return (claw.getPosition() - CLAW_OPEN) < EPSILON;
   }

   public Action latchOpen() { return new ActionUtil.ServoPositionAction(claw, CLAW_OPEN); }

   public Action latchClosed() {
      return new ActionUtil.ServoPositionAction(claw, CLAW_CLOSED);
   }

   public Action wristStored() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_STORED);
   }

   public Action wristScoring() {
      return new ActionUtil.ServoPositionAction(wrist, WRIST_SCORING);
   }
   public boolean isWristScoring() {
      return Math.abs(wrist.getPosition() - WRIST_SCORING) < EPSILON;
   }

   public Action mosaicAdjust() {
      return new ActionUtil.ServoPositionAction(mosaic, MOSAIC_ADJUSTING);
   }
   public Action mosaicClosed() {
      return new ActionUtil.ServoPositionAction(mosaic, MOSAIC_CLOSED);
   }

   public Action decreaseMosaicPos() {
      mosaicPosition = mosaic.getPosition() - 0.01;
      return new ActionUtil.ServoPositionAction(mosaic, mosaicPosition);
   }

   public Action increaseMosaicPos() {
      mosaicPosition = mosaic.getPosition() + 0.01;
      return new ActionUtil.ServoPositionAction(mosaic, mosaicPosition);
   }

   public boolean isSlideDown() {
      return slideSensor.isMagnetPresent() && Math.abs(slide.getVelocity()) < 3 && slide.getTargetPosition() < 5;
   }
   public boolean isTouchSensorPressed() {
      return touchSensor.isPressed();
   }
}
