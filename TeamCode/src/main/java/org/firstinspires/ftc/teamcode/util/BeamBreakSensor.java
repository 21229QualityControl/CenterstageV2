package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeamBreakSensor {
    private DigitalChannel sensor;

    public BeamBreakSensor(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.digitalChannel.get(deviceName);
    }

    public boolean isBeamBroken() {
        return !sensor.getState();
    }
}