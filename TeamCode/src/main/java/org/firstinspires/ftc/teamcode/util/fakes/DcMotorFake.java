package org.firstinspires.ftc.teamcode.util.fakes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A placeholder DCMotor
 * Some methods are simulated
 */
public class DcMotorFake extends HardwareDeviceFake implements DcMotorEx {
    private double power = 0;
    private int position = 0;
    private double velocity = 0;
    private int targetPositionTolerance = 5;
    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
    private RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;
    private MotorConfigurationType motorType = new MotorConfigurationType();
    private PIDFCoefficients pidfCoefficients = null;
    private boolean isEnergized = true;

    @Override
    public void setMotorEnable() {
        this.isEnergized = true;
    }

    @Override
    public void setMotorDisable() {
        this.isEnergized = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return this.isEnergized;
    }

    @Override
    public void setVelocity(double angularRate) {
        this.velocity = angularRate;
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(unit.toDegrees(angularRate) / 360 * 288);
    }

    @Override
    public double getVelocity() {
        return this.velocity;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(this.velocity / 288 * 360);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        this.pidfCoefficients = new PIDFCoefficients(pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        this.pidfCoefficients = pidfCoefficients;
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return new PIDCoefficients(this.pidfCoefficients.p, this.pidfCoefficients.i, this.pidfCoefficients.d);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return this.pidfCoefficients;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.targetPositionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return this.targetPositionTolerance;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motorType;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        this.motorType = motorType;
    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return -1;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return this.zeroPowerBehavior;
    }

    @Override
    public void setPowerFloat() {
        this.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        this.setPower(0.0);
    }

    @Override
    public boolean getPowerFloat() {
        return this.getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && this.getPower() == 0.0;
    }

    @Override
    public void setTargetPosition(int position) {
        this.position = position;
    }

    @Override
    public int getTargetPosition() {
        return this.position;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return this.position;
    }

    @Override
    public void setMode(RunMode mode) {
        this.runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return this.runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return this.direction;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return this.power * (this.direction.equals(Direction.FORWARD)? 1:-1);
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.power = 0;
        // this.position = 0;
        this.velocity = 0;
        this.targetPositionTolerance = 5;
        this.direction = Direction.FORWARD;
        this.zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
        this.runMode = RunMode.RUN_WITHOUT_ENCODER;
        this.motorType = new MotorConfigurationType();
        this.pidfCoefficients = null;
        this.isEnergized = true;
    }
}
