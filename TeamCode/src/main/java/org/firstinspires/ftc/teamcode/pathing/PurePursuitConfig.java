package org.firstinspires.ftc.teamcode.pathing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConfig {
    public static double xP = 0.05;
    public static double xD = 0.011;

    public static double yP = 0.05;
    public static double yD = 0.011;

    public static double hP = 1.1;
    public static double hD = 0.045;

    public static double END_GAIN = 5;

    public static double MAX_TRANSLATIONAL_SPEED = 1.0;
    public static double MAX_ROTATIONAL_SPEED = 0.9;
    public static double X_GAIN = 2.00;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = 0.03;

    public static double ACCEL_TIME = 0.5;
}