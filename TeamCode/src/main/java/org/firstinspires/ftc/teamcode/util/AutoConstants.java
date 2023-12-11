package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
   public static Pose2d[] blueScoring = {new Pose2d(44, 32, Math.toRadians(180)), new Pose2d(44, 36, Math.toRadians(180)), new Pose2d(44, 40, Math.toRadians(180))};
   public static Pose2d[] redScoring = {new Pose2d(43, -40, Math.toRadians(180)), new Pose2d(43, -34, Math.toRadians(180)), new Pose2d(43, -30, Math.toRadians(180))};
}
