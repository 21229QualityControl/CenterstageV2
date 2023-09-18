package org.firstinspires.ftc.teamcode.util.fakes;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * A placeholder Servo
 */
public class ServoFake extends HardwareDeviceFake implements Servo, PwmControl {
    private double position = 0;
    private Direction direction = Direction.FORWARD;
    private PwmRange pwmRange = new PwmRange(600, 2400);
    private boolean isPwmEnabled = true;

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return -1;
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
    public void setPosition(double position) {
        this.position = Range.clip(position, MIN_POSITION, MAX_POSITION);
    }

    @Override
    public double getPosition() {
        return this.position;
    }

    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public void setPwmRange(PwmRange range) {
        pwmRange = range;
    }

    @Override
    public PwmRange getPwmRange() {
        return pwmRange;
    }

    @Override
    public void setPwmEnable() {
        isPwmEnabled = true;
    }

    @Override
    public void setPwmDisable() {
        isPwmEnabled = false;
    }

    @Override
    public boolean isPwmEnabled() {
        return isPwmEnabled;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.position = 0;
        this.direction = Direction.FORWARD;
        this.pwmRange = new PwmRange(600, 2400);
        this.isPwmEnabled = true;
    }
}
