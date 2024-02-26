package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Class for our Blinkin LED strip
 *
 * Memory so we don't spam logs in case of constant pattern setting
 */
public class LED {
    private RevBlinkinLedDriver ledDriver;

    BlinkinPattern pattern = null;

    public LED(HardwareMap hardwareMap) {
        try {
            ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find Rev Blinkin Driver");
            this.ledDriver = null;
        }
        setPattern(BlinkinPattern.BLACK);
    }

    public void setPattern(BlinkinPattern pattern) {
        if (this.ledDriver == null) {
            return;
        }
        if (pattern != this.pattern) ledDriver.setPattern(pattern);
        this.pattern = pattern;
    }

    private class SetPatternAction implements Action {
        BlinkinPattern newPattern;

        public SetPatternAction(BlinkinPattern pattern) {
            this.newPattern = pattern;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (newPattern != pattern) ledDriver.setPattern(newPattern);
            pattern = newPattern;
            return false;
        }
    }

    public Action setPatternAction(BlinkinPattern pattern) {
        return new SetPatternAction(pattern);
    }

    public BlinkinPattern getPattern() {
        return pattern;
    }
}
