package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
//@Disabled
@TeleOp(group = "Test")
public class BeamBreakTest extends LinearOpMode {
   public static String NAME = "intakeBeam";
   private BeamBreakSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        sensor = new BeamBreakSensor(HardwareCreator.createDigitalChannel(hardwareMap, NAME));
        // Game loop
        while (!isStopRequested()) {
            telemetry.addData("Sensor", NAME);
            telemetry.addData("Beam Broken", sensor.isBeamBroken());
            telemetry.update();
        }
    }
}