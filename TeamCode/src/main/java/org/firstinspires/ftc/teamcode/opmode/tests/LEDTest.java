package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.LED;

/**
 * This is a simple test for finding a good LED pattern
 */
@Config
//@Disabled
@TeleOp(group = "Test")
public class LEDTest extends LinearOpMode {
    public static int PATTERN_INDEX = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LED led = new LED(hardwareMap);
        int maxIndex = RevBlinkinLedDriver.BlinkinPattern.values().length - 1;

        GamePadController g1 = new GamePadController(gamepad1);
        g1.update();

        telemetry.addData("Instructions", "Press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(PATTERN_INDEX));

            if (g1.aOnce()) PATTERN_INDEX = Math.min(PATTERN_INDEX + 1, maxIndex);
            if (g1.bOnce()) PATTERN_INDEX = Math.max(0, PATTERN_INDEX - 1);

            telemetry.addData("Color", led.getPattern());
            telemetry.addData("Index", PATTERN_INDEX + "/" + maxIndex);
            telemetry.update();

            g1.update();
        }
    }

}
