package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Hang {
   public static PIDCoefficients hangPID = new PIDCoefficients(0.0025, 0, 0.0004);
   public static int HANG_EXTENDED = 9300;
   final MotorWithPID hang;

   public Hang(HardwareMap hardwareMap) {
      this.hang = new MotorWithPID(HardwareCreator.createMotor(hardwareMap, "hang"), hangPID);
      this.hang.setMaxPower(1.0);
   }

   public Action extendHang() {
      return this.hang.setTargetPositionAction(HANG_EXTENDED);
   }
   public Action retractHang() {
      return this.hang.setTargetPositionAction(0);
   }
   public void update() {
      this.hang.update();
   }
}
