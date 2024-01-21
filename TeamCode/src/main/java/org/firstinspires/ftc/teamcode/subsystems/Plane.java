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
   public static double LATCH_SCORED = 0.56;
   public static double LATCH_CLOSED = 0.63;

   final Servo latch;

   public Plane(HardwareMap hardwareMap) {
      this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
   }

   public void initialize() {
      latch.setPosition(LATCH_CLOSED);
   }

   public Action scorePlane() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_SCORED);
   }
}
