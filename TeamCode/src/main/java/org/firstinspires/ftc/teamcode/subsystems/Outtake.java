package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Outtake {
   public static PIDCoefficients outtakePID;
   public static int OUTTAKE_EXTENDED = 100;
   public static int LATCH_OPEN = 1;
   public static int LATCH_CLOSED = 0;
   public static int WRIST_STORED = 0;
   public static int WRIST_SCORING = 1;
   final MotorWithPID slide;
   final Servo latch;
   final Servo wrist;

   public Outtake(HardwareMap hardwareMap) {
      this.slide = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "outtakeSlide"), outtakePID);
      this.latch = HardwareCreator.createServo(hardwareMap, "outtakeLatch");
      this.wrist = HardwareCreator.createServo(hardwareMap, "outtakeWrist");
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
