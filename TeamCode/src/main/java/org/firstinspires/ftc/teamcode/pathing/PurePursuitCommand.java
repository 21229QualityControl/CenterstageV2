package org.firstinspires.ftc.teamcode.pathing;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.ACCEL_TIME;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.X_GAIN;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.hD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.hP;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.xD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.xP;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.yD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.yP;

@Config
public class PurePursuitCommand implements Action {
    private final MecanumDrive drivetrain;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;


    public static PIDFController xController = new PIDFController(new PIDCoefficients(xP, 0.0, xD));
    public static PIDFController yController = new PIDFController(new PIDCoefficients(yP, 0.0, yD));
    public static PIDFController hController = new PIDFController(new PIDCoefficients(hP, 0.0, hD));

    private ElapsedTime accelLimit;

    private ElapsedTime timer;
    private double[] xPoints;
    private double[] yPoints;

    public PurePursuitCommand(MecanumDrive drivetrain, PurePursuitPath purePursuitPath) {
        this.drivetrain = drivetrain;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
        this.xPoints = purePursuitPath.xPoints();
        this.yPoints = purePursuitPath.yPoints();
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (accelLimit == null) accelLimit = new ElapsedTime();
        if (purePursuitPath.isFinished()) PID = true;

        PoseVelocity2d vel = drivetrain.updatePoseEstimate();
        Pose robotPose = new Pose(drivetrain.pose.position.x, drivetrain.pose.position.y, drivetrain.pose.heading.toDouble());
        Pose targetPose = purePursuitPath.update(robotPose);

        if (PID && timer == null) {
            timer = new ElapsedTime();
        }

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) finished = true;

        if (targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        // Drawing
        Canvas c = p.fieldOverlay();
        c.setStroke("#4CAF50");
        MecanumDrive.drawRobot(c, new Pose2d(targetPose.x, targetPose.y, targetPose.heading));

        c.setStroke("#3F51B5");
        MecanumDrive.drawRobot(c, new Pose2d(robotPose.x, robotPose.y, robotPose.heading));

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        // Do powers
        xController.setTargetPosition(targetPose.x);
        yController.setTargetPosition(targetPose.y);
        hController.setTargetPosition(targetPose.heading);
        double xPower = xController.update(robotPose.x);
        double yPower = yController.update(robotPose.y);
        double hPower = -hController.update(robotPose.heading);

        /*double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);*/
        double y_rotated = xPower * Math.cos(-robotPose.heading) + yPower * Math.sin(-robotPose.heading); // TODO: Check if -robotPose.heading is right
        double x_rotated = xPower * -Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        double accelScale = Math.min(accelLimit.seconds() / ACCEL_TIME, 1);
        drivetrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d(y_rotated, x_rotated * X_GAIN).times(accelScale),
                hPower*accelScale
        ));

        return !(PID && finished || (timer != null && timer.milliseconds() > 2000));
    }
}