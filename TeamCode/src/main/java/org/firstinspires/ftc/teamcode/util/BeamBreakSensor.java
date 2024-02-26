package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeamBreakSensor {
    private DigitalChannel sensor;

    public BeamBreakSensor(DigitalChannel channel) {
        sensor = channel;
    }

    public boolean isBeamBroken() {
        return !sensor.getState();
    }
}