package org.firstinspires.ftc.teamcode.pathing;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.pathing.Pose;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.pathing.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

@Config
public class PurePursuitCommand implements Action {
    private final MecanumDrive drivetrain;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static double xP = 0.0335;
    public static double xD = 0.006;

    public static double yP = 0.0335;
    public static double yD = 0.006;

    public static double hP = 1;
    public static double hD = 0.03;

    public static double kStatic = 0.05;

    public static PIDFController xController = new PIDFController(new PIDCoefficients(xP, 0, xD), 0.0, 0, 0);
    public static PIDFController yController = new PIDFController(new PIDCoefficients(yP, 0, yD), 0.0, 0, 0);
    public static PIDFController hController = new PIDFController(new PIDCoefficients(hP, 0, hD), 0.0, 0, 0);

    private ElapsedTime timer;

    public PurePursuitCommand(MecanumDrive drivetrain, PurePursuitPath purePursuitPath) {
        this.drivetrain = drivetrain;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (purePursuitPath.isFinished()) PID = true;

        PoseVelocity2d rawPose = drivetrain.updatePoseEstimate();
        Pose robotPose = new Pose(rawPose.component1().x, rawPose.component1().y, rawPose.component2());
        Pose targetPose = purePursuitPath.update(robotPose);

        if(PID && timer == null){
            timer = new ElapsedTime();
        }

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) finished = true;

        Canvas c = p.fieldOverlay();

        c.setStroke("#4CAF50");
        MecanumDrive.drawRobot(c, new Pose2d(targetPose.x, targetPose.y, targetPose.heading));

        c.setStroke("#3F51B5");
        MecanumDrive.drawRobot(c, new Pose2d(robotPose.x, robotPose.y, robotPose.heading));

        if (PID) {
            Pose delta = targetPose.subtract(robotPose);

            double xPower = xController.update(robotPose.x, targetPose.x);
            double yPower = yController.update(robotPose.y, targetPose.y);
            double hPower = -hController.update(0, delta.heading);

            double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
            double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

            if (Math.abs(x_rotated) < 0.01) x_rotated = 0;
            else x_rotated += kStatic * Math.signum(x_rotated);
            if (Math.abs(y_rotated) < 0.01) y_rotated = 0;
            else y_rotated += kStatic * Math.signum(y_rotated);
            if (Math.abs(hPower) < 0.01) hPower = 0;

            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(y_rotated, x_rotated), hPower));
        } else {
            Pose delta = targetPose.subtract(robotPose);
            double y_rotated = delta.x * Math.cos(robotPose.heading) - delta.y * Math.sin(robotPose.heading);
            double x_rotated = delta.x * Math.sin(robotPose.heading) + delta.y * Math.cos(robotPose.heading);

            double xPercentage = x_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
            double yPercentage = y_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
            double hPower = -hController.update(0, delta.heading);

            Log.d("DELTAX", String.valueOf(delta.x));
            Log.d("DELTAY", String.valueOf(delta.y));
            Log.d("DELTAXROTATED", String.valueOf(x_rotated));
            Log.d("DELTAYROTATED", String.valueOf(y_rotated));

            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(yPercentage, xPercentage * 1.6), hPower));
        }

        return !(PID && finished || (timer != null && timer.milliseconds() > 2000));
    }
}