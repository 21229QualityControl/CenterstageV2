package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class WolfpackDrive {
    public static double maxVelocityX = 50; // max positive straight velocity. Record using MaxVelStraightTest.
    public static double maxVelocityY = 50; // max positive sideways velocity. Record using MaxVelStrafeTest.
    public static double centripetalWeighting = 0.02; // adjust by trial and error for how much smoothing you need. Wolfpack calculates it but I can't be bothered.
    public static double dashboardVectorScale = 1;

    private Vector2d leftFrontWheelForceVector;  // these vectors should not change during the match,
    private Vector2d leftBackWheelForceVector;   // but are left non-final for modifying maxVelocities
    private Vector2d rightBackWheelForceVector;
    private Vector2d rightFrontWheelForceVector;
    private double wheelForceVectorMag;

    public Vector2d centripetalCircleCenterDrawn = null;
    public Vector2d centripetalCircleRadiusDrawn = null;
    public Vector2d centripetalVectorDrawn = null;
    public Vector2d robotDriveDirectionDrawn = null;

    private final MecanumDrive mecanumDrivebase;
    private final RingBuffer positionBuffer = new RingBuffer(3);

    public WolfpackDrive(MecanumDrive mecanumDrivebase) {
        this.mecanumDrivebase = mecanumDrivebase;
        updateWheelForceVectors();
    }

    public void updateWheelForceVectors() {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        leftBackWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        rightBackWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        rightFrontWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        wheelForceVectorMag = getAbsMax(leftFrontWheelForceVector.norm(), leftBackWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    public void setDrivePowers(Vector2d joystickPower, double turnPower, PoseVelocity2d currentVelocity) {
        // Find centripetal power
        Vector2d centripetalPower = calculateCentripetalPower(currentVelocity);

        // Combine with joystick power
        double maxPower = joystickPower.norm();
        Vector2d combinedPower = centripetalPower.plus(joystickPower);

        // Combine and scale down (is this needed? We already scale after turning)
        Vector2d scaledCombinedPower = combinedPower.div(combinedPower.norm());
        if (scaledCombinedPower.norm() > maxPower) { // scale down if over max. Never scale up
            scaledCombinedPower = scaledCombinedPower.div(scaledCombinedPower.norm()).times(maxPower);
        }
        robotDriveDirectionDrawn = scaledCombinedPower.times(dashboardVectorScale);

        // Rotate wheel force vectors to drive direction
        double driveDirection = scaledCombinedPower.angleCast().log();
        double drivePower = scaledCombinedPower.norm();
        Vector2d newLeftFrontWheelForceVector = leftFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newLeftBackWheelForceVector = leftBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newRightBackWheelForceVector = rightBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newRightFrontWheelForceVector = rightFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);

        // scale down extraneous strafe
        if (Math.abs(newLeftFrontWheelForceVector.y) > Math.abs(newRightFrontWheelForceVector.y)) {
            newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(Math.abs(newRightFrontWheelForceVector.y) / Math.abs(newLeftFrontWheelForceVector.y));
        } else if (Math.abs(newLeftFrontWheelForceVector.y) < Math.abs(newRightFrontWheelForceVector.y)) {
            newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(Math.abs(newLeftFrontWheelForceVector.y) / Math.abs(newRightFrontWheelForceVector.y));
        }
        if (Math.abs(newLeftBackWheelForceVector.y) > Math.abs(newRightBackWheelForceVector.y)) {
            newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(Math.abs(newRightBackWheelForceVector.y) / Math.abs(newLeftBackWheelForceVector.y));
        } else if (Math.abs(newLeftBackWheelForceVector.y) < Math.abs(newRightBackWheelForceVector.y)) {
            newRightBackWheelForceVector = newRightBackWheelForceVector.times(Math.abs(newLeftBackWheelForceVector.y) / Math.abs(newRightBackWheelForceVector.y));
        }

        // get actual motor powers
        double leftFrontPower = newLeftFrontWheelForceVector.norm() / wheelForceVectorMag;
        double leftBackPower = newLeftBackWheelForceVector.norm() / wheelForceVectorMag;
        double rightBackPower = newRightBackWheelForceVector.norm() / wheelForceVectorMag;
        double rightFrontPower = newRightFrontWheelForceVector.norm() / wheelForceVectorMag;

        // consider turn power
        leftFrontPower = leftFrontPower - turnPower;
        leftBackPower = leftBackPower - turnPower;
        rightBackPower = rightBackPower + turnPower;
        rightFrontPower = rightFrontPower + turnPower;
        maxPower = getAbsMax(maxPower, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
        }

        // set motor powers
        mecanumDrivebase.leftFront.setPower(leftFrontPower);
        mecanumDrivebase.leftBack.setPower(leftBackPower);
        mecanumDrivebase.rightBack.setPower(rightBackPower);
        mecanumDrivebase.rightFront.setPower(rightFrontPower);
    }

    public void trackPosition(Pose2d pose) {
        positionBuffer.put(pose);
    }

    private Vector2d calculateCentripetalPower(PoseVelocity2d currentVelocity) {
        // Read the three points from ring buffer
        Vector2d p3 = positionBuffer.read(0).position; // y(t=3)
        Vector2d p2 = positionBuffer.read(1).position; // y(t=2)
        Vector2d p1 = positionBuffer.read(2).position; // y(t=1)

        // Find circumcenter
        double ax = p1.x;
        double ay = p1.y;
        double bx = p2.x;
        double by = p2.y;
        double cx = p3.x;
        double cy = p3.y;
        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
        Vector2d circumcenter = new Vector2d(ux, uy);

        if (Double.isFinite(circumcenter.x) && Double.isFinite(circumcenter.y)) {
            centripetalCircleCenterDrawn = circumcenter.times(1);

            Vector2d circleVector = circumcenter.minus(p3); // represents vector from p3 to the center of the circle
            centripetalCircleRadiusDrawn = circleVector.times(1);

            Vector2d centripetalVector = circleVector.div(circleVector.sqrNorm()); // scales circleVector with mag r to centripetalVector with mag 1/r

            // Calculate power
            double power = centripetalWeighting * centripetalVector.norm() * currentVelocity.linearVel.sqrNorm(); // F=mv^2/r, power = weight*F

            Vector2d centripetalPower = centripetalVector.div(centripetalVector.norm()).times(power); // change magnitude to power
            centripetalVectorDrawn = centripetalPower.times(dashboardVectorScale);

            return centripetalPower;
        } else {
            centripetalCircleCenterDrawn = null;
            centripetalCircleRadiusDrawn = null;
            centripetalVectorDrawn = null;
            return new Vector2d(0, 0);
        }
    }

    private double getAbsMax(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }
}