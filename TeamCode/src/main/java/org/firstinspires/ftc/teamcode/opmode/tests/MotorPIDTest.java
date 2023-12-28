package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

/**
 * This is a test program for testing any motor under any run mode
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class MotorPIDTest extends LinearOpMode {
   public static String NAME = "outtakeSlide";
   public static PIDCoefficients pid = new PIDCoefficients(0.004, 0, 0.0004);
   public static int TARGET_POSITION = 0;
   private MotorWithPID motor;

   @Override
   public void runOpMode() throws InterruptedException {

      // Communicate to dashboard
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      motor = new MotorWithPID(hardwareMap.get(DcMotorEx.class, NAME), pid);
      motor.setMaxPower(1.0);
      telemetry.addData("Motor", NAME);
      telemetry.update();

      waitForStart();

      while (opModeIsActive()) {
         telemetry.addData("Real", motor.getCurrentPosition());
         telemetry.addData("Target", motor.getTargetPosition());
         telemetry.update();

         motor.setTargetPosition(TARGET_POSITION);
         motor.update();
      }
   }
}
