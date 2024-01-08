package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.HardwareCreator;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
        public static class Params {
                /*public double parYTicks = 6.5 / (24.0 / 44593.0); // y position of the parallel encoder (in tick units)
                public double perpXTicks = -5 / (24.0 / 44593.0); // x position of the perpendicular encoder (in tick units)*/
                public double parYTicks = 11110.24332364755; // y position of the parallel encoder (in tick units)
                public double perpXTicks = -9980.239241262898; // x position of the perpendicular encoder (in tick units)
        }

        public static Params PARAMS = new Params();

        public final Encoder par, perp;
        public final IMU imu;

        private int lastParPos, lastPerpPos;
        private Rotation2d lastHeading;

        private final double inPerTick;

        private double lastRawHeadingVel, headingVelOffset;

        public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
                par = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "leftFront")));
                par.setDirection(DcMotorSimple.Direction.REVERSE);
                perp = new OverflowEncoder(new RawEncoder(HardwareCreator.createMotor(hardwareMap, "rightFront")));
                this.imu = imu;

                lastParPos = par.getPositionAndVelocity().position;
                lastPerpPos = perp.getPositionAndVelocity().position;
                lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

                this.inPerTick = inPerTick;

                FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
        }

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        private double getHeadingVelocity() {
                double rawHeadingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
                        headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
                }
                lastRawHeadingVel = rawHeadingVel;
                Log.d("HEADING", String.valueOf(rawHeadingVel));
                return headingVelOffset + rawHeadingVel;
        }

        public Twist2dDual<Time> update() {
                PositionVelocityPair parPosVel = par.getPositionAndVelocity();
                PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
                Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

                int parPosDelta = parPosVel.position - lastParPos;
                int perpPosDelta = perpPosVel.position - lastPerpPos;
                double headingDelta = heading.minus(lastHeading);
                double headingVel = getHeadingVelocity();

                Twist2dDual<Time> twist = new Twist2dDual<>(
                                new Vector2dDual<>(
                                                new DualNum<Time>(new double[] {
                                                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                                                }).times(inPerTick),
                                                new DualNum<Time>(new double[] {
                                                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                                                }).times(inPerTick)),
                                new DualNum<>(new double[] {
                                                headingDelta,
                                                headingVel,
                                }));

                lastParPos = parPosVel.position;
                lastPerpPos = perpPosVel.position;
                lastHeading = heading;

                return twist;
        }
}
