package org.firstinspires.ftc.teamcode.opmode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This is a test program for testing any motor under any run mode
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class MotorVelocityPIDTest extends LinearOpMode {
   public static String NAME = "intakeMotor";
   public static PIDCoefficients pid = new PIDCoefficients(0.00005, 0, 0);
   public static int TARGET_VELOCITY = 1600;
   private MotorWithVelocityPID motor;

   @Override
   public void runOpMode() throws InterruptedException {

      // Communicate to dashboard
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      motor = new MotorWithVelocityPID(hardwareMap.get(DcMotorEx.class, NAME), pid);
      motor.setMaxPower(1.0);
      telemetry.addData("Motor", NAME);
      telemetry.update();

      waitForStart();

      while (opModeIsActive()) {
         telemetry.addData("Real", motor.getVelocity());
         telemetry.addData("Target", motor.getTargetVelocity());
         telemetry.update();

         motor.setTargetVelocity(TARGET_VELOCITY);
         motor.update();
      }
   }
}
