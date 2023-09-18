package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This is a test program for tuning any servo's positions.
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class ServoTest extends LinearOpMode {
    public static String NAME = "servo";
    public static boolean REVERSED = false;
    public static HardwareCreator.ServoType SERVO_TYPE = HardwareCreator.ServoType.DEFAULT;
    public static double POSITION = 0.5;

    private List<String> servoNames;
    private static int servoIndex = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pull a list of all the servos from the configuration
        servoNames = new ArrayList<>(hardwareMap.servo.size());
        hardwareMap.servo.entrySet().forEach(entry -> servoNames.add(entry.getKey()));
        Collections.sort(servoNames);
        if (!servoNames.contains(NAME)) servoIndex = -1; // reset index if dashboard changed it

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        // Get input
        while (opModeInInit()) {
            // Control servo selection
            g1.update();
            if (g1.leftBumperOnce()) {
                servoIndex = Range.clip(servoIndex - 1, 0, servoNames.size() - 1);
                NAME = servoNames.get(servoIndex);
            }
            if (g1.rightBumperOnce()) {
                servoIndex = Range.clip(servoIndex + 1, 0, servoNames.size() - 1);
                NAME = servoNames.get(servoIndex);
            }
            // Control reverse
            if (g1.aOnce()) REVERSED = !REVERSED;
            // Control init position
            double joystick = -g1.right_stick_y;
            if (Math.abs(joystick) > 0.01) POSITION = Range.clip(POSITION + 0.0001*Math.pow(joystick, 3), 0, 1);
            if (g1.rightStickButtonOnce()) POSITION = 0.5;

            // Telemetry state and instructions
            telemetry.addLine("~ Use bumpers to scroll through servo list");
            telemetry.addLine("~ Use A to reverse the servo");
            telemetry.addLine("~ Use right joystick to adjust the init position");
            telemetry.addLine();
            if (!servoNames.contains(NAME)) telemetry.addLine("*** THIS SERVO DOES NOT EXIST ***");
            telemetry.addData("Servo name", NAME);
            telemetry.addData("Servo direction", (REVERSED ? "REVERSE" : "FORWARD"));
            telemetry.addData("Servo type", SERVO_TYPE);
            telemetry.addData("Init Position", POSITION);
            telemetry.update();
        }

        // Exit immediately if stopped
        if (isStopRequested()) return;

        // Create the servo
        Servo servo = HardwareCreator.createServo(hardwareMap, NAME, SERVO_TYPE);
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        // Game loop
        while (!isStopRequested()) {
            // Update controller
            g1.update();

            // Control position
            double joystick = -g1.right_stick_y;
            if (Math.abs(joystick) > 0.01) POSITION = Range.clip(POSITION + 0.001*Math.pow(joystick, 3), 0, 1);
            if (g1.dpadUpOnce()) POSITION = Range.clip(POSITION + 0.01, 0, 1);
            if (g1.dpadDownOnce()) POSITION = Range.clip(POSITION - 0.01, 0, 1);

            // Set position
            servo.setPosition(POSITION);

            // Telemetry state and instructions
            telemetry.addLine("~ Adjust servo position with the dpad or right joystick");
            telemetry.addLine();
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Servo name", NAME);
            telemetry.addData("Servo direction", (REVERSED ? "REVERSE" : "FORWARD"));
            telemetry.addData("Servo type", SERVO_TYPE);
            telemetry.update();
        }
    }
}