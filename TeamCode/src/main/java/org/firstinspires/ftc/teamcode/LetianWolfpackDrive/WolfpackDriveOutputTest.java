package org.firstinspires.ftc.teamcode.LetianWolfpackDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WolfpackDriveOutputTest {

    // Run isolated. Might need to run with coverage.
    public static void main(String[] args)
    {
        System.out.println();
        updateWheelForceVectors();

        test(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 150, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 180, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 210, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 270, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 330, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        test(1, 90, 0, 0, 0, 0, 0, 0, 12, 0, 30);
        test(0, 90, 0, 0, 0, 0, 0, 0, 12, 0, 30);
//        test(0.5, 180, 0, 30, 30, 0, 0, 16, 12, 30, 30);

    }

    public static double maxVelocityX = 50; // max positive straight velocity. Record using MaxVelStraightTest.
    public static double maxVelocityY = 50; // max positive sideways velocity. Record using MaxVelStrafeTest.
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

        double[] wheelPowers = driveWithCorrection(new PoseVelocity2d(stick, turnPower), velocity);

        System.out.println("Ring buffer: " + positionBuffer);
        if (centripetalCircleCenterDrawn == null) System.out.println("Circle stats: No circle");
        else System.out.println(String.format("Circle stats: %s", getCircleString()));
        System.out.println(String.format("Stick:           %s", formatVector(stick)));
        System.out.println(String.format("Current vel:     %s", formatVector(velocity.linearVel)));
        System.out.println(String.format("Correction:      %s", getCorrectionString()));
        System.out.println(String.format("Drive direction: %s", getDriveDirectionString()));
        System.out.println(getWheelPowerString(wheelPowers));

        double actualDriveDirection = Math.toDegrees(leftFrontWheelForceVector.times(wheelPowers[0]).plus(leftBackWheelForceVector.times(wheelPowers[1])).plus(rightBackWheelForceVector.times(wheelPowers[2])).plus(rightFrontWheelForceVector.times(wheelPowers[3])).angleCast().log());
        System.out.println(String.format("Actual drive direction: %5.1f°, %s", actualDriveDirection, Math.abs(AngleUnit.normalizeDegrees(actualDriveDirection-stickAngle)) <= 1e-6?"OK":"ERROR"));

        System.out.println();
    }

    /**
     * Flashes new maxVelocities into the force vectors.
     */
    public static void updateWheelForceVectors() {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        leftBackWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        rightBackWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        rightFrontWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        wheelForceVectorMag = getAbsMax(leftFrontWheelForceVector.norm(), leftBackWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    /**
     * Sets wheel powers depending on given robot drive direction
     */
    public static double[] setDrivePowers(PoseVelocity2d powers) {
        // Rotate wheel force vectors to drive direction
        double driveDirection = powers.linearVel.angleCast().log();
        double drivePower = powers.linearVel.norm();
        Vector2d newLeftFrontWheelForceVector = leftFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newLeftBackWheelForceVector = leftBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newRightBackWheelForceVector = rightBackWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        Vector2d newRightFrontWheelForceVector = rightFrontWheelForceVector.angleCast().plus(-driveDirection).vec().times(drivePower);
        double leftFrontPower = drivePower;
        double leftBackPower = drivePower;
        double rightBackPower = drivePower;
        double rightFrontPower = drivePower;

        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftFrontWheelForceVector.x, newLeftFrontWheelForceVector.y, newRightFrontWheelForceVector.x, newRightFrontWheelForceVector.y));
        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftBackWheelForceVector.x, newLeftBackWheelForceVector.y, newRightBackWheelForceVector.x, newRightBackWheelForceVector.y));

        // scale down extraneous strafe
        double frontNegScale = -Math.signum(newLeftFrontWheelForceVector.y * newRightFrontWheelForceVector.y);
        if (Math.abs(newLeftFrontWheelForceVector.y) >= Math.abs(newRightFrontWheelForceVector.y)) {
            System.out.println("Scale down leftFront");
            double pairedY = newRightFrontWheelForceVector.y;
            double selfY = newLeftFrontWheelForceVector.y;
            double factor = frontNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(factor);
            leftFrontPower = leftFrontPower * factor;
        } else {
            System.out.println("Scale down rightFront");
            double pairedY = newLeftFrontWheelForceVector.y;
            double selfY = newRightFrontWheelForceVector.y;
            double factor = frontNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(factor);
            rightFrontPower = rightFrontPower * factor;
        }
        double backNegScale = -Math.signum(newLeftBackWheelForceVector.y * newRightBackWheelForceVector.y);
        if (Math.abs(newLeftBackWheelForceVector.y) > Math.abs(newRightBackWheelForceVector.y)) {
            System.out.println("Scale down leftBack");
            double pairedY = newRightBackWheelForceVector.y;
            double selfY = newLeftBackWheelForceVector.y;
            double factor = backNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(factor);
            leftBackPower = leftBackPower * factor;
        } else {
            System.out.println("Scale down rightBack");
            double pairedY = newLeftBackWheelForceVector.y;
            double selfY = newRightBackWheelForceVector.y;
            double factor = backNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newRightBackWheelForceVector = newRightBackWheelForceVector.times(factor);
            rightBackPower = rightBackPower * factor;
        }

        // Correct signs if majority is negative x
        if (newLeftFrontWheelForceVector.x + newLeftBackWheelForceVector.x + newRightBackWheelForceVector.x + newRightFrontWheelForceVector.x < 0) {
            newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(-1);
            newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(-1);
            newRightBackWheelForceVector = newRightBackWheelForceVector.times(-1);
            newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(-1);
            leftFrontPower = -leftFrontPower;
            leftBackPower = -leftBackPower;
            rightBackPower = -rightBackPower;
            rightFrontPower = -rightFrontPower;
        }

        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftFrontWheelForceVector.x, newLeftFrontWheelForceVector.y, newRightFrontWheelForceVector.x, newRightFrontWheelForceVector.y));
        System.out.println(String.format("(%+7.2f, %7.2f)  (%+7.2f, %7.2f)", newLeftBackWheelForceVector.x, newLeftBackWheelForceVector.y, newRightBackWheelForceVector.x, newRightBackWheelForceVector.y));

        Vector2d wheelForceSum = newLeftFrontWheelForceVector.plus(newLeftBackWheelForceVector).plus(newRightBackWheelForceVector).plus(newRightFrontWheelForceVector);
        System.out.println(String.format("Adding wheel forces: mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f), %s", wheelForceSum.norm(), Math.toDegrees(wheelForceSum.angleCast().log()), wheelForceSum.x, wheelForceSum.y, Math.abs(Math.toDegrees(wheelForceSum.angleCast().log()))<=1e-6?"OK":"ERROR"));

        // consider turn power
        leftFrontPower = leftFrontPower - powers.angVel;
        leftBackPower = leftBackPower - powers.angVel;
        rightBackPower = rightBackPower + powers.angVel;
        rightFrontPower = rightFrontPower + powers.angVel;
        double maxPower = getAbsMax(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
        }

        return new double[]{ leftFrontPower, leftBackPower, rightBackPower, rightFrontPower };
    }

    /**
     * Drives with additional centripetal correction
     */
    public static double[] driveWithCorrection(PoseVelocity2d powers, PoseVelocity2d currentVelocity) {
        // Find centripetal power
        Vector2d centripetalPower = calculateCentripetalPower(currentVelocity);

        // Combine with joystick power
        Vector2d combinedPower = combinePower(powers.linearVel, centripetalPower);

        robotDriveDirectionDrawn = combinedPower.times(dashboardVectorScale);

        return setDrivePowers(new PoseVelocity2d(combinedPower, powers.angVel));
    }

    /**
     * Adds two power vectors together, ensuring that the result is not of bigger magnitude than the mainPower
     */
    private static Vector2d combinePower(Vector2d mainPower, Vector2d correctivePower) {
        // get max power
        double maxPower = mainPower.norm();

        // Sum
        Vector2d combinedPower = mainPower.plus(correctivePower);

        // Scale down
        Vector2d scaledCombinedPower = combinedPower;
        if (combinedPower.norm() > maxPower) { // scale down if over max. Never scale up
            scaledCombinedPower = combinedPower.div(combinedPower.norm()).times(maxPower);
        }

        return scaledCombinedPower;
    }

    /**
     * Calculates the centripetal vector by fitting 3 past positions and considering current linear velocity
     */
    private static Vector2d calculateCentripetalPower(PoseVelocity2d currentVelocity) {
        // Read the three points from ring buffer
        if (!positionBuffer.isFull()) return new Vector2d(0, 0);
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

    /**
     * Returns the largest magnitude of the given values
     */
    private static double getAbsMax(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }

    public static String formatVector(Vector2d vector) {
        if (vector == null) return "";
        return String.format("mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", vector.norm(), Math.toDegrees(vector.angleCast().log()), vector.x, vector.y);
    }

    public static String getCircleString() {
        if (centripetalCircleCenterDrawn == null) {
            return "No circle";
        } else {
            return String.format("r=%.2f, center=(%.2f, %.2f)", centripetalCircleRadiusDrawn.norm(), centripetalCircleCenterDrawn.x, centripetalCircleCenterDrawn.y);
        }
    }

    public static String getCorrectionString() {
        if (centripetalVectorDrawn == null) {
            return "None";
        } else {
            return formatVector(centripetalVectorDrawn);
        }
    }

    public static String getDriveDirectionString() {
        if (robotDriveDirectionDrawn == null) {
            return "Stopped";
        } else {
            return formatVector(robotDriveDirectionDrawn);
        }
    }

    public static String getWheelPowerString(double[] wheelPowers) {
        return String.format("%+1.3f  %+1.3f\n%+1.3f  %+1.3f", wheelPowers[0], wheelPowers[3], wheelPowers[1], wheelPowers[2]);
    }
}