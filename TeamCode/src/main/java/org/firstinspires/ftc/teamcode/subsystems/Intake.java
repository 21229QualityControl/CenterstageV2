package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = 1600; // Max speed is 2400
   public static int INTAKE_REVERSE_SPEED = -2400;

   final MotorWithVelocityPID intakeMotor;
   final Servo intakeWrist;

   final BeamBreakSensor backBeam;
   final BeamBreakSensor frontBeam;

   public static double WRIST_STORED = 0;
   public static double WRIST_DOWN = 1;
   public static double[] STACK_POSITIONS = {
           0, // Getting 1
           0, // Getting 2
           0, // Getting 3
           0, // Getting 4
           // Use WRIST_DOWN to get 5
   };

   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
         this.intakeWrist = HardwareCreator.createServo(hardwareMap, "intakeWrist");
         this.backBeam = new BeamBreakSensor(hardwareMap, "backBeam");
         this.frontBeam = new BeamBreakSensor(hardwareMap, "frontBeam");
   }

   public void initialize() {
      this.intakeWrist.setPosition(WRIST_STORED);
   }

   public Action intakeOn() {
      return intakeMotor.setTargetVelocityAction(INTAKE_SPEED);
   }

   public Action intakeReverse() {
      return intakeMotor.setTargetVelocityAction(INTAKE_REVERSE_SPEED);
   }

   public boolean isIntaking() {
      return intakeMotor.getTargetVelocity() == INTAKE_SPEED;
   }
   public boolean isReversing() {
      return intakeMotor.getTargetVelocity() == INTAKE_REVERSE_SPEED;
   }

   public Action intakeOff() {
      intakeMotor.resetIntegralGain();
      return intakeMotor.setTargetVelocityAction(0);
   }
   public void update() {
      intakeMotor.update();
   }

   public Action wristStored() {
      return new ActionUtil.ServoPositionAction(intakeWrist, WRIST_STORED);
   }

   public Action wristDown() {
      return new ActionUtil.ServoPositionAction(intakeWrist, WRIST_DOWN);
   }

   public Action wristStack(int numberIntaked) {
      return new ActionUtil.ServoPositionAction(intakeWrist, STACK_POSITIONS[numberIntaked]);
   }

   public int pixelCount() {
      int count = 0;
      if (backBeam.isBeamBroken()) {
         count++;
      }
      if (frontBeam.isBeamBroken()) {
         count++;
      }
      return count;
   }
}
