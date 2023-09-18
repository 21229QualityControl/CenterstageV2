package org.firstinspires.ftc.teamcode.opmode.tests;

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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This is a test program for testing any motor under any run mode
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class MotorTest extends LinearOpMode {
    public static String NAME = "motor";
    public static boolean REVERSED = false;
    public static boolean BRAKE = false;
    public static Mode MODE = Mode.POWER;
    public static double VELOCITY = 0;
    public static int POSITION = 0;
    public static double POWER = 0;
//    public static MotorPIDValues PID = new MotorPIDValues();

    private List<String> motorNames;
    private static int motorIndex = -1;

    public enum Mode {
        POWER("RUN_WITHOUT_ENCODER (TO_POWER)"),
        VELOCITY("RUN_USING_ENCODER (TO_VELOCITY)"),
        POSITION("RUN_TO_POSITION (TO_POSITION)");

        String description;

        Mode(String description) {
            this.description = description;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pull a list of all the motors from the configuration
        motorNames = new ArrayList<>(hardwareMap.servo.size());
        hardwareMap.dcMotor.entrySet().forEach(entry -> motorNames.add(entry.getKey()));
        Collections.sort(motorNames);
        if (!motorNames.contains(NAME)) motorIndex = -1; // reset index if dashboard changed it

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        // Get input
        while (opModeInInit()) {
            // Control motor selection
            g1.update();
            if (g1.leftBumperOnce()) {
                motorIndex = Range.clip(motorIndex - 1, 0, motorNames.size() - 1);
                NAME = motorNames.get(motorIndex);
            }
            if (g1.rightBumperOnce()) {
                motorIndex = Range.clip(motorIndex + 1, 0, motorNames.size() - 1);
                NAME = motorNames.get(motorIndex);
            }
            // Control reverse
            if (g1.aOnce()) REVERSED = !REVERSED;
            // Control mode
            if (g1.xOnce()) MODE = Mode.POWER;
            if (g1.yOnce()) MODE = Mode.VELOCITY;
            if (g1.bOnce()) MODE = Mode.POSITION;

            // Telemetry state and instructions
            telemetry.addLine("~ Use bumpers to scroll through motor list");
            telemetry.addLine("~ Use A to reverse the motor");
            telemetry.addLine("~ Use [X-Power / Y-Velocity / B-Position] to change modes");
            telemetry.addLine();
            if (!motorNames.contains(NAME)) telemetry.addLine("*** THIS MOTOR DOES NOT EXIST ***");
            telemetry.addData("Motor name", NAME);
            telemetry.addData("Motor direction", (REVERSED ? "REVERSE" : "FORWARD"));
            telemetry.addData("Motor mode", MODE.description);
            telemetry.addData("Motor zero power behavior", BRAKE ? "BRAKE" : "FLOAT");
            telemetry.update();
        }

        // exit immediately if stopped
        if (isStopRequested()) return;

        // Create the motor
        DcMotorEx motor = HardwareCreator.createMotor(hardwareMap, NAME);
        motor.setPower(0);
        motor.setTargetPosition(0);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Game loop
        while (!isStopRequested()) {
            // Update controller
            g1.update();

            // Control mode
            if (g1.xOnce()) {
                MODE = Mode.POWER;
                POWER = 0;
            }
            if (g1.yOnce()) {
                MODE = Mode.VELOCITY;
                VELOCITY = 0;
            }
            if (g1.bOnce()) {
                MODE = Mode.POSITION;
                POSITION = motor.getCurrentPosition();
                POWER = 0;
            }
            // Control primary
            double leftJoystick = -g1.left_stick_y;
            if (Math.abs(leftJoystick) > 0.01) {
                if (MODE == Mode.POWER) POWER = Range.clip(POWER + 0.1*Math.pow(leftJoystick, 3), -1, 1);
                if (MODE == Mode.VELOCITY) VELOCITY = VELOCITY + 10*Math.pow(leftJoystick, 3);
                if (MODE == Mode.POSITION) POSITION = POSITION + (int)(10*Math.pow(leftJoystick, 3));
            }
            int dpadChange = 0;
            if (g1.dpadUpOnce()) dpadChange = 1;
            else if (g1.dpadDownOnce()) dpadChange = -1;
            else if (g1.dpadRightOnce()) dpadChange = 10;
            else if (g1.dpadLeftOnce()) dpadChange = -10;
            if (dpadChange != 0) {
                if (MODE == Mode.POWER) POWER = Range.clip(POWER + 0.01*dpadChange, -1, 1);
                if (MODE == Mode.VELOCITY) VELOCITY = VELOCITY + 10*dpadChange;
                if (MODE == Mode.POSITION) POSITION = POSITION + (int)(10*Math.pow(leftJoystick, 3));
            }
            // Control secondary
            double rightJoystick = -g1.right_stick_y;
            if (Math.abs(rightJoystick) > 0.01 && MODE == Mode.POSITION) {
                POWER = Range.clip(Math.abs(POWER) + rightJoystick, 0, 1);
            }
            // Stop system
            if (g1.aOnce()) {
                if (MODE == Mode.POWER) POWER = 0;
                if (MODE == Mode.VELOCITY) VELOCITY = 0;
                if (MODE == Mode.POSITION) POWER = 0;
            }

            // Telemetry instructions
            telemetry.addLine("~ Use [X-Power / Y-Velocity / B-Position] to change modes");
            telemetry.addLine("~ Use A to stop the motor");
            telemetry.addLine("~ Use Left joystick or dpad to adjust the primary value [" + (MODE) + "]");
            telemetry.addLine("~ Use Right joystick to adjust the secondary value [" + (MODE == Mode.POSITION ? "POWER" : "N/A") + "]");
            telemetry.addLine();

            // Apply brake settings
            if (BRAKE) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Run modes
            switch (MODE) {
                case POWER:
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(POWER);

                    telemetry.addData("Mode", MODE.description);
                    telemetry.addData("Applied Power", motor.getPower());
                    telemetry.addData("-Position", motor.getCurrentPosition());
                    telemetry.addData("-Velocity", motor.getVelocity());
                    break;
                case VELOCITY:
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocity(VELOCITY, AngleUnit.DEGREES);

                    telemetry.addData("Mode", MODE.description);
                    telemetry.addData("Applied Velocity Target", VELOCITY);
                    telemetry.addData("-Velocity", motor.getVelocity());
                    telemetry.addData("-Position", motor.getCurrentPosition());
                    break;
                case POSITION:
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setTargetPosition(POSITION);
                    motor.setPower(POWER);

                    telemetry.addData("Mode", MODE.description);
                    telemetry.addData("Applied Position Target", motor.getTargetPosition());
                    telemetry.addData("Applied Max Power", motor.getPower());
                    telemetry.addData("-Position", motor.getCurrentPosition());
                    telemetry.addData("-Velocity", motor.getVelocity());
                    break;
            }

//            motor.setPositionPIDFCoefficients(PID.POS_P);
//            motor.setVelocityPIDFCoefficients(PID.VEL_P, PID.VEL_I, PID.VEL_D, PID.VEL_F);

            telemetry.update();
        }
    }

//    public static class MotorPIDValues {
//        public double POS_P = 0; // don't use
//        public double VEL_P = 12;
//        public double VEL_I = 0.9;
//        public double VEL_D = 0;
//        public double VEL_F = 0;
//    }
}
