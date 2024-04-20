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
   public static double LATCH_SCORED = 0.32;
   public static double LATCH_CLOSED = 0.36;
   public static double LATCH_PUSH = 1;
//   public static double WRIST_STORED = 0.47;
//   public static double WRIST_SCORED = 0.34;

   final Servo latch;
//   final Servo wrist;

   public Plane(HardwareMap hardwareMap) {
      this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
//      this.wrist = HardwareCreator.createServo(hardwareMap, "planeWrist");
   }

   public void initialize() {
      latch.setPosition(LATCH_CLOSED);
//      wrist.setPosition(WRIST_STORED);
   }

   public Action scorePlane() {
      return new SequentialAction(
              /*new ActionUtil.ServoPositionAction(latch, LATCH_PUSH),
              new SleepAction(0.4),*/
              new ActionUtil.ServoPositionAction(latch, LATCH_SCORED),
              new SleepAction(0.3),
              new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED)
      );
   }
}
