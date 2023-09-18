package org.firstinspires.ftc.teamcode.util.fakes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * A placeholder CRServo
 */
public class CRServoFake extends HardwareDeviceFake implements CRServo {
    private double power = 0;
    private Direction direction = Direction.FORWARD;

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
    public void setPower(double power) {
        this.power = Range.clip(-1, power, 1);
    }

    @Override
    public double getPower() {
        return this.power;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.power = 0;
        this.direction = Direction.FORWARD;
    }
}
