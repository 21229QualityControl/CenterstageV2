package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.MagnetSwitchSensor;

@Config
//@Disabled
@TeleOp(group = "Test")
public class MagnetSensorTest extends LinearOpMode {
    public static String NAME = "outtakeMagnetSwitch";
    private MagnetSwitchSensor slideSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        slideSensor = new MagnetSwitchSensor(hardwareMap, NAME);
        // Game loop
        while (!isStopRequested()) {
            telemetry.addData("Sensor", NAME);
            telemetry.addData("Is magnet present", slideSensor.isMagnetPresent());
            telemetry.update();
        }
    }
}