package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
        public static class Params {
                public double par0YTicks = -1600.4302597829123; // y position of the first parallel encoder (in tick units)
                public double par1YTicks = 1589.5159695593209; // y position of the second parallel encoder (in tick units)
                public double perpXTicks = -427.34187393510723; // x position of the perpendicular encoder (in tick units)
        }

        public static Params PARAMS = new Params();

        public final Encoder par0, par1, perp;

        public final double inPerTick;

        public final IMU imu;

        private int lastPar0Pos, lastPar1Pos, lastPerpPos;
        private boolean initialized;

        private double lastRawHeadingVel, headingVelOffset;
        private Rotation2d lastHeading;

        public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, IMU imu) {
                // TODO: make sure your config has **motors** with these names (or change them)
                //   the encoders should be plugged into the slot matching the named motor
                //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
                par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
                par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
                perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
                this.imu = imu;

                // TODO: reverse encoder directions if needed
                par1.setDirection(DcMotorSimple.Direction.REVERSE);

                this.inPerTick = inPerTick;

                FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
        }

        public Twist2dDual<Time> update() {
                PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
                PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
                PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
                Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

                // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
                double rawHeadingVel = angularVelocity.zRotationRate;
                if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
                        headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
                }
                lastRawHeadingVel = rawHeadingVel;
                double headingVel = headingVelOffset + rawHeadingVel;

                if (!initialized) {
                        initialized = true;

                        lastPar0Pos = par0PosVel.position;
                        lastPar1Pos = par1PosVel.position;
                        lastPerpPos = perpPosVel.position;
                        lastHeading = heading;

                        return new Twist2dDual<>(
                                Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                                DualNum.constant(0.0, 2)
                        );
                }

                int par0PosDelta = par0PosVel.position - lastPar0Pos;
                int par1PosDelta = par1PosVel.position - lastPar1Pos;
                int perpPosDelta = perpPosVel.position - lastPerpPos;
                double headingDelta = heading.minus(lastHeading);
                lastHeading = heading;
                if (Math.abs(headingVel) < Math.toRadians(0)) { // Use dead wheels when heading change low low
                        headingDelta = (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
                        headingVel =  (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
                }

                Twist2dDual<Time> twist = new Twist2dDual<>(
                        new Vector2dDual<>(
                                new DualNum<Time>(new double[] {
                                        (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                        (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                }).times(inPerTick),
                                new DualNum<Time>(new double[] {
                                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                                }).times(inPerTick)
                        ),
                        new DualNum<>(new double[] {
                                headingDelta,
                                headingVel,
                        })
                );

                lastPar0Pos = par0PosVel.position;
                lastPar1Pos = par1PosVel.position;
                lastPerpPos = perpPosVel.position;

                return twist;
        }
}