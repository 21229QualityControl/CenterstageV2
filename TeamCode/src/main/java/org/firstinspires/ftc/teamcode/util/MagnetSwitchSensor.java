package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class MagnetSwitchSensor {
    private DigitalChannel sensor;

    public MagnetSwitchSensor(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.digitalChannel.get(deviceName);
    }

    public boolean isMagnetPresent() {
        return !sensor.getState();
    }
}
