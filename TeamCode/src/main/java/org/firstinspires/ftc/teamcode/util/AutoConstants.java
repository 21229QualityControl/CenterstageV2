package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
   public static Pose2d[] blueScoring = {
           new Pose2d(43, 28, Math.toRadians(180)),
           new Pose2d(43, 36, Math.toRadians(180)),
           new Pose2d(43, 40, Math.toRadians(180))
   };
   public static Pose2d[] redScoring = {
           new Pose2d(43, -41, Math.toRadians(180)),
           new Pose2d(43, -36, Math.toRadians(180)),
           new Pose2d(43, -29, Math.toRadians(180))
   };
}
