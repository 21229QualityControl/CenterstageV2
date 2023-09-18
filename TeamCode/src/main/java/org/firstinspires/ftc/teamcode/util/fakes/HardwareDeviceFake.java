package org.firstinspires.ftc.teamcode.util.fakes;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * A placeholder HardwareDevice
 */
public abstract class HardwareDeviceFake implements HardwareDevice {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return this.getClass().getSimpleName();
    }

    @Override
    public String getConnectionInfo() {
        return "Fake connection";
    }

    @Override
    public int getVersion() {
        return -1;
    }

    @Override
    public void close() {

    }
}
