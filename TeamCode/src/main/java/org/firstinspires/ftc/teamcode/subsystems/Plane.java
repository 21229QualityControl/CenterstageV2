package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public class Plane {
   public static double LATCH_SCORED = 0.47;
   public static double LATCH_CLOSED = 0.54;
   public static double WRIST_STORED = 0.45;
   public static double WRIST_SCORED = 0.38;
   public static double WRIST_SCORED_BIG = 0.34;
   public static double WRIST_SCORED_HUMONGOUS = 0.29;

   final Servo latch;
   final Servo wrist;

   public Plane(HardwareMap hardwareMap) {
      this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
      this.wrist = HardwareCreator.createServo(hardwareMap, "planeWrist");
   }

   public void initialize() {
      latch.setPosition(LATCH_CLOSED);
      wrist.setPosition(WRIST_STORED);
   }

   public Action scorePlane(double angle) {
//      return new ActionUtil.ServoPositionAction(latch, LATCH_SCORED);
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(wrist, angle),
              new SleepAction(0.5),
              new ActionUtil.ServoPositionAction(latch, LATCH_SCORED)
      );
   }
   public Action storePlane() {
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(wrist, WRIST_STORED),
              new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED)
      );
   }
}
