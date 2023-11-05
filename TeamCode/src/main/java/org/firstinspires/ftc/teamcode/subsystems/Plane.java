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
   public static double LATCH_SCORED = 0.6;
   public static double LATCH_CLOSED = 0.87;
   public static double HOLDER_OPEN = 0.15;
   public static double HOLDER_CLOSED = 0.30;

   final Servo latch;
   final Servo holder;

   public Plane(HardwareMap hardwareMap) {
      this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
      this.holder = HardwareCreator.createServo(hardwareMap, "planeHolder");
   }

   public void initialize() {
      latch.setPosition(LATCH_CLOSED);
      holder.setPosition(HOLDER_CLOSED);
   }

   public Action scorePlane() {
      return new SequentialAction(
              new ActionUtil.ServoPositionAction(holder, HOLDER_OPEN),
              new SleepAction(0.4),
              new ActionUtil.ServoPositionAction(latch, LATCH_SCORED)
      );
   }
}
