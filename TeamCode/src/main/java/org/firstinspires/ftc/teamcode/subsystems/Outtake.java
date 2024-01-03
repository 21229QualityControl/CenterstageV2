package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MagnetSwitchSensor;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID = new PIDCoefficients(0.004, 0.0015, 0.0004);
   public static int OUTTAKE_TELEOP = 0;
   public static int OUTTAKE_MIDLOW = 500;
   public static int OUTTAKE_MID = 800;
   public static int OUTTAKE_LOW = 300;
   public static double LATCH_SCORING = 0.55;
   public static double LATCH_OPEN = 0.8;
   public static double LATCH_CLOSED = 0.9;
   public static double WRIST_STORED = 0.74;
   public static double WRIST_SCORING = 1;

   public static double MOSAIC_ADJUSTING = 0.54;
   public static double MOSAIC_CLOSED = 0.11;

   public double mosaicPosition;
   final MotorWithPID slide;
   public boolean slidePIDEnabled = true;
   final Servo latch;
   final Servo wrist;
   public ServoImplEx mosaic;

   final MagnetSwitchSensor slideSensor;

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
      this.slideSensor = new MagnetSwitchSensor(hardwareMap, "outtakeMagnetSensor");
      this.mosaic = (ServoImplEx) HardwareCreator.createServo(hardwareMap, "mosaic");
   }
   // The object outtake can get the slide motor to be directly used in Manual Drive.
   public MotorWithPID getSlide(){
      return this.slide;
   }
   // The object outtake can get the mosaic servo to be directly used in Manual Drive.
   public ServoImplEx getServo() {return this.mosaic; }

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
      OUTTAKE_TELEOP = OUTTAKE_MIDLOW;
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
      slide.getMotor().setPower(power);
   }
   public void lockPosition() {
      OUTTAKE_TELEOP = this.slide.getCurrentPosition();
      this.slide.setTargetPosition(OUTTAKE_TELEOP);
   }

   public Action extendOuttakeMidBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_MID);
   }

   public Action extendOuttakeMid() {
      return this.slide.setTargetPositionAction(OUTTAKE_MID);
   }


   public Action extendOuttakeTeleopBlocking() {
      return this.slide.setTargetPositionActionBlocking(OUTTAKE_TELEOP);
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

   public void disableMosaicServoPwm() {
      this.mosaic.setPwmDisable();
   }

   public boolean isSlideMagnetPresent() {
      return slideSensor.isMagnetPresent();
   }

   public boolean isSlideDown() {
      return slideSensor.isMagnetPresent() && Math.abs(slide.getVelocity()) < 3 && slide.getTargetPosition() < 5;
   }

   public void zeroMotorInternals() {
      this.slide.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      this.slide.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }
}
