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
   public static int LATCH_SCORED;
   public static int LATCH_CLOSED;

   final Servo latch;

   public Plane(HardwareMap hardwareMap) {
         this.latch = HardwareCreator.createServo(hardwareMap, "planeLatch");
   }

   public Action latchClosed() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_CLOSED);
   }

   public Action latchScored() {
      return new ActionUtil.ServoPositionAction(latch, LATCH_SCORED);
   }
}
