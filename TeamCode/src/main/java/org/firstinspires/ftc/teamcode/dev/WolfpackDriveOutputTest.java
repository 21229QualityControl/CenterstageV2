package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public class WolfpackDriveOutputTest {

    // Run isolated. Might need to run with coverage.
    public static void main(String[] args)
    {
        System.out.println();
        updateWheelForceVectors();

        test(1, 30, 0, 30, 90, 0, 0, 10, 10, 12, 20);
        test(1, 30, 0, 30, 90, 0, 0, 0, 12, 0, 30);
        test(0.5, 180, 0, 30, 30, 0, 0, 16, 12, 30, 30);

    }

    public static double maxVelocityX = 50; // max straight velocity. Record using MaxVelStraightTest.
    public static double maxVelocityY = 50; // max right velocity. Record using MaxVelStrafeTest.
    public static double centripetalWeighting = 0.02; // adjust by trial and error for how much smoothing you need. Wolfpack calculates it but I can't be bothered.
    public static double dashboardVectorScale = 1;

    private static Vector2d leftFrontWheelForceVector;  // these vectors should not change during the match,
    private static Vector2d leftBackWheelForceVector;   // but are left non-final for modifying maxVelocities
    private static Vector2d rightBackWheelForceVector;
    private static Vector2d rightFrontWheelForceVector;
    private static double wheelForceVectorMag;

    private static Vector2d centripetalCircleCenterDrawn = null;
    private static Vector2d centripetalCircleRadiusDrawn = null;
    private static Vector2d centripetalVectorDrawn = null;
    private static Vector2d robotDriveDirectionDrawn = null;

    private static final RingBuffer positionBuffer = new RingBuffer(3);

    public static void test(double stickPower, double stickAngle, double turnPower, double velPower, double velAngle, double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {

        positionBuffer.put(new Pose2d(p3x, p3y, 0));
        positionBuffer.put(new Pose2d(p2x, p2y, 0));
        positionBuffer.put(new Pose2d(p1x, p1y, 0));

        Vector2d stick = Rotation2d.Companion.exp(Math.toRadians(stickAngle)).vec().times(stickPower);
        PoseVelocity2d velocity = new PoseVelocity2d(Rotation2d.Companion.exp(Math.toRadians(velAngle)).vec().times(velPower), 0);

        double[] wheelPowers = getDrivePowers(stick, turnPower, velocity);

        System.out.println("Ring buffer: " + positionBuffer);
        if (centripetalCircleCenterDrawn == null) System.out.println("Circle stats: No circle");
        else System.out.println(String.format("Circle stats: r=%.2f, center=(%.2f, %.2f)", centripetalCircleRadiusDrawn.norm(), centripetalCircleCenterDrawn.x, centripetalCircleCenterDrawn.y));
        System.out.println(String.format("Stick:           mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", stickPower, stickAngle, stick.x, stick.y));
        System.out.println(String.format("Current vel:     mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", velPower, velAngle, velocity.linearVel.x, velocity.linearVel.y));
        System.out.println(String.format("Correction:      mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", centripetalVectorDrawn!=null?centripetalVectorDrawn.norm():0, centripetalVectorDrawn!=null?Math.toDegrees(centripetalVectorDrawn.angleCast().log()):0, centripetalVectorDrawn!=null?centripetalVectorDrawn.x:0, centripetalVectorDrawn!=null?centripetalVectorDrawn.y:0));
        System.out.println(String.format("Drive direction: mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", robotDriveDirectionDrawn.norm(), Math.toDegrees(robotDriveDirectionDrawn.angleCast().log()), robotDriveDirectionDrawn.x, robotDriveDirectionDrawn.y));
        System.out.println(String.format("%+1.3f  %+1.3f", wheelPowers[0], wheelPowers[3]));
        System.out.println(String.format("%+1.3f  %+1.3f", wheelPowers[1], wheelPowers[2]));
        System.out.println();
    }

    public static void updateWheelForceVectors() {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        leftBackWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        rightBackWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        rightFrontWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        wheelForceVectorMag = getAbsMax(leftFrontWheelForceVector.norm(), leftBackWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    public static double[] getDrivePowers(Vector2d joystickPower, double turnPower, PoseVelocity2d currentVelocity) {
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

        return new double[]{ leftFrontPower, leftBackPower, rightBackPower, rightFrontPower };
    }

    private static Vector2d calculateCentripetalPower(PoseVelocity2d currentVelocity) {
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

    private static double getAbsMax(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }
}