package org.firstinspires.ftc.teamcode.util;


import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

import kotlin.jvm.functions.Function3;

public class MotorWithPID {
    private DcMotorEx motor;
    private PIDFController pidfController;
    private PIDCoefficients pid;
    private int targetPosition = 0;
    private int internalOffset = 0;
    private int tolerance = 100;
    private double maxPower = 0;

    public MotorWithPID(DcMotorEx motor, PIDCoefficients pid) {
        this(motor, pid, (t, x, v) -> 0.0);
    }

    public MotorWithPID(DcMotorEx motor, PIDCoefficients pid, Function3<Double, Double, Double, Double> f) {
        this.motor = motor;
        this.pid = pid;
        this.pidfController = new PIDFController(pid, 0, 0, 0, f);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * Updates the power sent to the motor according to the pidf controller.
     */
    public void update() {
        double newPower = Range.clip(this.pidfController.update(motor.getCurrentPosition(), motor.getVelocity()), -maxPower, maxPower);
//        Log.d("MotorWithPID", "newPower " + newPower + ", lastError " + pidfController.getLastError());
        motor.setPower(newPower);
    }

    /**
     * Updates the power sent to the motor according to the pidf controller,
     * but scale pid output with a current and target voltage
     */
    public void update(double currentVoltage, double targetVoltage) {
        double newPower = Range.clip(this.pidfController.update(motor.getCurrentPosition(), motor.getVelocity()) * targetVoltage / currentVoltage, -maxPower, maxPower);
//        Log.d("MotorWithPID", "newPower " + newPower + ", lastError " + pidfController.getLastError());
        motor.setPower(newPower);
    }

    /**
     * Update the PID values in the controller.
     * Note that it is not possible to replace f after instantiation
     * @param newPID the new pid values to use
     */
    public void setPIDCoefficients(PIDCoefficients newPID) {
        this.pid.kP = newPID.kP;
        this.pid.kI = newPID.kI;
        this.pid.kD = newPID.kD;
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold there at. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * @param position the desired encoder target position
     */
    public void setTargetPosition(int position) {
        this.targetPosition = position;
        this.pidfController.setTargetPosition(position - internalOffset); // TODO: Verify sign
    }

    private class TargetPositionAction implements Action {
        int position;
        boolean blocking;

        public TargetPositionAction(int position, boolean blocking) {
            this.position = position;
            this.blocking = blocking;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (getTargetPosition() != position) {
                setTargetPosition(position);
                if (blocking) {
                    return true;
                }
            }
            if (blocking) {
                Log.d("BLOCKING", isBusy() ? "true" : "false");
                return isBusy();
            }
            return false;
        }
    }

    /**
     * Creates an action that will call setTargetPosition with the provided position
     *
     * @param position the desired encoder target position
     */
    public Action setTargetPositionAction(int position) {
        return new TargetPositionAction(position, false);
    }

    /**
     * Creates an action that will call setTargetPosition with the provided position and
     * wait for the position to be reached
     *
     * @param position the desired encoder target position
     */
    public Action setTargetPositionActionBlocking(int position) {
        return new TargetPositionAction(position, true);
    }

    /**
     * Returns the current reading of the encoder for this motor in encoder ticks.
     * @return the current reading of the encoder for this motor
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() + internalOffset;
    }

    public void zeroMotorInternals() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetIntegralGain() {
        this.pidfController.reset();
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    public boolean isBusy() {
        return Math.abs(pidfController.getLastError()) > tolerance || Math.abs(pidfController.getTargetVelocity()) > tolerance;
    }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Sets the maximum power level that can be sent to the motor
     * @param maxPower the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    /**
     * Returns the maximum power level that can be sent to the motor
     * @return the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public double getMaxPower() {
        return maxPower;
    }

    /**
     * Returns the current power level sent to the motor.
     * @return the current level of the motor, a value in the interval [-1.0, 1.0]
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Sets the motor power to zero
     */
    public void stopMotor() {
        motor.setPower(0);
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Remaps the current position to the given position
     * @param currentPosition the position to remap as
     */
    public void setCurrentPosition(int currentPosition) {
        this.internalOffset = currentPosition - motor.getCurrentPosition(); // raw + offset = current
    }

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    public void setTargetPositionTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    public int getTargetPositionTolerance() {
        return tolerance;
    }
}
