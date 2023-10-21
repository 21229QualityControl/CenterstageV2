package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public class Plane {
   public static double LATCH_SCORED = 0.95;
   public static double LATCH_CLOSED = 0.45;

   final Servo latch;

   public Plane(HardwareMap hardwareMap) {
         this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
   }

   public void initialize() {
      latch.setPosition(LATCH_CLOSED);
   }

   public Action latchClosed() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED);
   }

   public Action latchScored() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_SCORED);
   }
}
