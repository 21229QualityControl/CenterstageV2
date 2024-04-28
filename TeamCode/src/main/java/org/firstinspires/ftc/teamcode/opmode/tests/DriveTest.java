package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
public class DriveTest extends LinearOpMode {
    public static double DRIVE_X = 0;
    public static double DRIVE_Y = 0;
    public static double DRIVE_TURN = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Communicate to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pull a list of all the motors from the configuration
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Create our controller interface
        GamePadController g1 = new GamePadController(gamepad1);

        telemetry.addData("leftFrontCurrent", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightFrontCurrent", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightBackCurrent", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("leftBackCurrent", drive.leftBack.getCurrent(CurrentUnit.AMPS));

        // Get input
        waitForStart();

        // exit immediately if stopped
        if (isStopRequested()) return;

        // Game loop
        while (!isStopRequested()) {
            // Update controller
            g1.update();

            // Control mode
            if (g1.left_stick_y != 0) {
                DRIVE_X = -g1.left_stick_y;
            }
            if (g1.left_stick_x != 0) {
                DRIVE_Y = -g1.left_stick_x;
            }
            if (g1.left_trigger != 0 || g1.right_trigger != 0) {
                DRIVE_TURN = g1.left_trigger - g1.right_trigger;
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(DRIVE_X, DRIVE_Y), DRIVE_TURN));

            // Telemetry
            telemetry.addData("leftFrontCurrent", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFrontCurrent", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightBackCurrent", drive.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftBackCurrent", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
