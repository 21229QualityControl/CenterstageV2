package org.firstinspires.ftc.teamcode.LetianWolfpackDrive;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.GamePadController;

@TeleOp(group = "Test")
public class WolfpackDriveTest extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double SLOW_TURN_SPEED = 0.3;

    private GamePadController g1;
    private MecanumDrive baseDrive;
    private WolfpackDrive drive;
    private ActionScheduler sched;

    private double speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Init
        g1 = new GamePadController(gamepad1);
        g1.update();
        sched = new ActionScheduler();
        baseDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive = new WolfpackDrive(baseDrive);

        sched.queueAction(telemetryPacket -> {
            Canvas c = telemetryPacket.fieldOverlay();
            Pose2d pose = baseDrive.pose;
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, pose);
            if (drive.centripetalVectorDrawn != null) {
                c.setStroke("#E8412E");
                c.strokeLine(pose.position.x, pose.position.y, pose.position.x + drive.centripetalVectorDrawn.x, pose.position.y + drive.centripetalVectorDrawn.y);
            }
            if (drive.centripetalCircleCenterDrawn != null && drive.centripetalCircleRadiusDrawn != null) {
                c.setStroke("#F56251");
                c.strokeCircle(drive.centripetalCircleCenterDrawn.x, drive.centripetalCircleCenterDrawn.y, drive.centripetalCircleRadiusDrawn.norm());
            }
            if (drive.robotDriveDirectionDrawn != null) {
                c.setStroke("#41A8E8");
                c.strokeLine(pose.position.x, pose.position.y, pose.position.x + drive.robotDriveDirectionDrawn.x, pose.position.y + drive.robotDriveDirectionDrawn.y);
            }
            return true; // keep running
        });

        // Ready!
        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        // On start
        if (opModeIsActive()) {
            resetRuntime();
            g1.reset();
        }

        // Main loop
        while (opModeIsActive()) {
            g1.update();

            if (g1.xOnce()) speed = speed > 0.5 ? 1 : 0.5;

            double input_x = Math.pow(-g1.left_stick_y, 3) * speed;
            double input_y = Math.pow(-g1.left_stick_x, 3) * speed;
            Vector2d input = new Vector2d(input_x, input_y);

            double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
            if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
            if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

            PoseVelocity2d currentVel = baseDrive.updatePoseEstimate();
            drive.trackPosition(baseDrive.pose);
            drive.driveWithCorrection(new PoseVelocity2d(input, input_turn), currentVel);

            telemetry.addData("Circle stats", drive.getCircleString());
            telemetry.addData("Stick", drive.formatVector(input));
            telemetry.addData("Current vel", drive.formatVector(currentVel.linearVel));
            telemetry.addData("Correction", drive.getCorrectionString());
            telemetry.addData("Drive direction", drive.getDriveDirectionString());
            telemetry.addLine(drive.getWheelPowerString());
            telemetry.update();

            sched.update();
        }
    }
}