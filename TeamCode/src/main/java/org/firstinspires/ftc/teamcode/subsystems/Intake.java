package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

@Config
public class Intake {
   public static int INTAKE_SPEED = -800; // Max speed is 2400
   public static int INTAKE_REVERSE_SPEED = 2400;

   final MotorWithVelocityPID intakeMotor;
   final Servo intakeWristLeft;
   final Servo intakeWristRight;
   final Servo intakeFeed;

   final BeamBreakSensor backBeam;
   final BeamBreakSensor frontBeam;

   public static double WRIST_LEFT_STORED = 0.17;
   public static double WRIST_LEFT_DOWN = 0.44;
   public static double[] WRIST_LEFT_STACK_POSITIONS = {
           0.33, // Getting 1
           0.36, // Getting 2
           0.39, // Getting 3
           0.42, // Getting 4
           // Use WRIST_DOWN to get 5
   };

   public static double WRIST_RIGHT_STORED = 0.34;
   public static double WRIST_RIGHT_DOWN = 0.08;
   public static double[] WRIST_RIGHT_STACK_POSITIONS = {
           0.18, // Getting 1
           0.16, // Getting 2
           0.14, // Getting 3
           0.1, // Getting 4
           // Use WRIST_DOWN to get 5
   };

   public static double FEED_CLOSED = 0.69; // Nice
   public static double FEED_OPEN = 0.85;

   public static PIDCoefficients intakeMotorPid = new PIDCoefficients(0.00005, 0, 0);

   public Intake(HardwareMap hardwareMap) {
         this.intakeMotor = new MotorWithVelocityPID(HardwareCreator.createMotor(hardwareMap, "intakeMotor"), intakeMotorPid);
         this.intakeMotor.setMaxPower(1.0);
         this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         this.intakeWristLeft = HardwareCreator.createServo(hardwareMap, "intakeWristLeft");
         this.intakeWristRight = HardwareCreator.createServo(hardwareMap, "intakeWristRight");
         this.intakeFeed = HardwareCreator.createServo(hardwareMap, "intakeFeed");
         this.backBeam = new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, "backBeam"));
         this.frontBeam =new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, "frontBeam"));
   }

   public void initialize() {
      this.intakeWristLeft.setPosition(WRIST_LEFT_STORED);
      this.intakeWristRight.setPosition(WRIST_RIGHT_STORED);
      this.intakeFeed.setPosition(FEED_CLOSED);
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
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_STORED), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_STORED));
   }
   public boolean wristIsStored() {
      return Math.abs(intakeWristLeft.getPosition() - WRIST_LEFT_STORED) < EPSILON;
   }

   public Action wristDown() {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_DOWN), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_DOWN));
   }

   public Action wristStack(int numberIntaked) {
      return new ParallelAction(new ActionUtil.ServoPositionAction(intakeWristLeft, WRIST_LEFT_STACK_POSITIONS[numberIntaked]), new ActionUtil.ServoPositionAction(intakeWristRight, WRIST_RIGHT_STACK_POSITIONS[numberIntaked]));
   }

   public Action feedClosed() {
      return new ActionUtil.ServoPositionAction(intakeFeed, FEED_CLOSED);
   }

   public Action feedOpen() {
      return new ActionUtil.ServoPositionAction(intakeFeed, FEED_OPEN);
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
