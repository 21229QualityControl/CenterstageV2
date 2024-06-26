package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "Test")
@Config
public class LocalizationTest extends LinearOpMode {
    public static double POWER_X = 0;
    public static double POWER_Y = 0;
    public static double POWER_TURN = 0;
    public static boolean USE_DASHBOARD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if (USE_DASHBOARD) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER_X, POWER_Y), POWER_TURN));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x),
                        -gamepad1.right_stick_x));
            }
            PoseVelocity2d vel = drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(Math.atan2(drive.pose.heading.imag, drive.pose.heading.real)));
            telemetry.addData("dx", vel.linearVel.x);
            telemetry.addData("dy", vel.linearVel.y);
            telemetry.addData("dheading", Math.toDegrees(vel.angVel));
            telemetry.update();
        }
    }
}
