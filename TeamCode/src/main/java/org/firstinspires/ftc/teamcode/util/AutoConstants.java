package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
   public static Pose2d[] blueScoring = {new Pose2d(42, 32, Math.toRadians(180)), new Pose2d(42, 36, Math.toRadians(180)), new Pose2d(42, 40, Math.toRadians(180))};
   public static Pose2d[] redScoring = {new Pose2d(40, -38, Math.toRadians(180)), new Pose2d(40, -32, Math.toRadians(180)), new Pose2d(40, -28, Math.toRadians(180))};
}
