package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.ACCEL_TIME;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.END_GAIN;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.X_GAIN;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.hD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.hP;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.xD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.xP;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.yD;
import static org.firstinspires.ftc.teamcode.pathing.PurePursuitConfig.yP;

import android.util.Log;

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

@Config
public class WaitPositionCommand implements Action {
    private final MecanumDrive drivetrain;
    private final double position;
    private final boolean x;
    private final boolean lt;

    public WaitPositionCommand(MecanumDrive drivetrain, double position, boolean lt, boolean x) {
        this.drivetrain = drivetrain;
        this.position = position;
        this.lt = lt;
        this.x = x;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        double val;
        if (x) {
            val = drivetrain.pose.position.x;
        } else {
            val = drivetrain.pose.position.y;
        }
        if (lt) {
            return val < position;
        }
        return val > position;
    }
}