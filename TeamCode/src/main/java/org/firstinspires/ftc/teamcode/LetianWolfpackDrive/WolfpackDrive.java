package org.firstinspires.ftc.teamcode.LetianWolfpackDrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class WolfpackDrive {
    public static double maxVelocityX = 77; // max positive straight velocity. Record using MaxVelStraightTest.
    public static double maxVelocityY = 51; // max positive sideways velocity. Record using MaxVelStrafeTest.
    public static double centripetalWeighting = 0.001; // adjust by trial and error for how much smoothing you need. Wolfpack calculates it but I can't be bothered.
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

    /**
     * Flashes new maxVelocities into the force vectors.
     */
    public void updateWheelForceVectors() {
        leftFrontWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        leftBackWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        rightBackWheelForceVector = new Vector2d(maxVelocityX, -maxVelocityY);
        rightFrontWheelForceVector = new Vector2d(maxVelocityX, maxVelocityY);
        wheelForceVectorMag = getAbsMax(leftFrontWheelForceVector.norm(), leftBackWheelForceVector.norm(), rightBackWheelForceVector.norm(), rightFrontWheelForceVector.norm());
    }

    /**
     * Sets wheel powers depending on given robot drive direction
     */
    public void setDrivePowers(PoseVelocity2d powers) {
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

        // scale down extraneous strafe
        double frontNegScale = -Math.signum(newLeftFrontWheelForceVector.y * newRightFrontWheelForceVector.y);
        if (Math.abs(newLeftFrontWheelForceVector.y) >= Math.abs(newRightFrontWheelForceVector.y)) {
            double pairedY = newRightFrontWheelForceVector.y;
            double selfY = newLeftFrontWheelForceVector.y;
            double factor = frontNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newLeftFrontWheelForceVector = newLeftFrontWheelForceVector.times(factor);
            leftFrontPower = leftFrontPower * factor;
        } else {
            double pairedY = newLeftFrontWheelForceVector.y;
            double selfY = newRightFrontWheelForceVector.y;
            double factor = frontNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newRightFrontWheelForceVector = newRightFrontWheelForceVector.times(factor);
            rightFrontPower = rightFrontPower * factor;
        }
        double backNegScale = -Math.signum(newLeftBackWheelForceVector.y * newRightBackWheelForceVector.y);
        if (Math.abs(newLeftBackWheelForceVector.y) > Math.abs(newRightBackWheelForceVector.y)) {
            double pairedY = newRightBackWheelForceVector.y;
            double selfY = newLeftBackWheelForceVector.y;
            double factor = backNegScale * Math.abs(pairedY / selfY);
            if (!Double.isFinite(factor)) factor = 0;
            newLeftBackWheelForceVector = newLeftBackWheelForceVector.times(factor);
            leftBackPower = leftBackPower * factor;
        } else {
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

        // set motor powers
        mecanumDrivebase.leftFront.setPower(leftFrontPower);
        mecanumDrivebase.leftBack.setPower(leftBackPower);
        mecanumDrivebase.rightBack.setPower(rightBackPower);
        mecanumDrivebase.rightFront.setPower(rightFrontPower);
    }

    /**
     * Drives with additional centripetal correction
     */
    public void driveWithCorrection(PoseVelocity2d powers, PoseVelocity2d currentVelocity) {
        // Find centripetal power
        Vector2d centripetalPower = calculateCentripetalPower(currentVelocity);

        // Combine with joystick power
        Vector2d combinedPower = combinePower(powers.linearVel, centripetalPower);

        robotDriveDirectionDrawn = combinedPower.times(dashboardVectorScale);

        setDrivePowers(new PoseVelocity2d(combinedPower, powers.angVel));
    }

    /**
     * Adds two power vectors together, ensuring that the result is not of bigger magnitude than the mainPower
     */
    private Vector2d combinePower(Vector2d mainPower, Vector2d correctivePower) {
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
     * Adds a new position into ring buffer for centripetal calculation
     */
    public void trackPosition(Pose2d pose) {
        positionBuffer.put(pose);
    }

    /**
     * Calculates the centripetal vector by fitting 3 past positions and considering current linear velocity
     */
    private Vector2d calculateCentripetalPower(PoseVelocity2d currentVelocity) {
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
    private double getAbsMax(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }

    public String formatVector(Vector2d vector) {
        if (vector == null) return "";
        return String.format("mag=%5.1f, angle=%5.1f°, (%5.2f, %5.2f)", vector.norm(), Math.toDegrees(vector.angleCast().log()), vector.x, vector.y);
    }

    public String getCircleString() {
        if (centripetalCircleCenterDrawn == null) {
            return "No circle";
        } else {
            return String.format("r=%.2f, center=(%.2f, %.2f)", centripetalCircleRadiusDrawn.norm(), centripetalCircleCenterDrawn.x, centripetalCircleCenterDrawn.y);
        }
    }

    public String getCorrectionString() {
        if (centripetalVectorDrawn == null) {
            return "None";
        } else {
            return formatVector(centripetalVectorDrawn);
        }
    }

    public String getDriveDirectionString() {
        if (robotDriveDirectionDrawn == null) {
            return "Stopped";
        } else {
            return formatVector(robotDriveDirectionDrawn);
        }
    }

    public String getWheelPowerString() {
        return String.format("%+1.3f  %+1.3f\n%+1.3f  %+1.3f", mecanumDrivebase.leftFront.getPower(), mecanumDrivebase.rightFront.getPower(), mecanumDrivebase.leftBack.getPower(), mecanumDrivebase.rightBack.getPower());
    }
}