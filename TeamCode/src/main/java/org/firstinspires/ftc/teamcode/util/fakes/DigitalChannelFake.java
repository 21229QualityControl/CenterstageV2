package org.firstinspires.ftc.teamcode.util.fakes;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

public class DigitalChannelFake extends HardwareDeviceFake implements DigitalChannel {
    private Mode mode;

    @Override
    public Mode getMode() {
        return mode;
    }

    @Override
    public void setMode(Mode mode) {
        this.mode = mode;
    }

    @Override
    public boolean getState() {
        return false;
    }

    @Override
    public void setState(boolean state) {

    }

    @Override
    public void setMode(DigitalChannelController.Mode mode) {

    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }
}