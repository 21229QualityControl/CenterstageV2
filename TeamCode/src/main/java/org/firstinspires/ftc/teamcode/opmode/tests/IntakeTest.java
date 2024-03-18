package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

/**
 * This is a test program for testing any motor under any run mode
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class IntakeTest extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
      MotorWithVelocityPID intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), new PIDCoefficients(0.00005, 0, 0));
      intakeMotor.setMaxPower(1.0);
      Servo intakeWristLeft = HardwareCreator.createServo(hardwareMap, "intakeWristLeft");
      Servo intakeWristRight = HardwareCreator.createServo(hardwareMap, "intakeWristRight");

      waitForStart();

      while (opModeIsActive()) {
         intakeMotor.setTargetVelocity(800);
         intakeMotor.update();

         intakeWristLeft.setPosition(0.43);
         intakeWristRight.setPosition(0.09);
      }
   }
}
