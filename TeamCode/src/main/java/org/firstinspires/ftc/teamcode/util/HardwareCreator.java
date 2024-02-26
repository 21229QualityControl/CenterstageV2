package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.fakes.CRServoFake;
import org.firstinspires.ftc.teamcode.util.fakes.DcMotorFake;
import org.firstinspires.ftc.teamcode.util.fakes.DigitalChannelFake;
import org.firstinspires.ftc.teamcode.util.fakes.ServoFake;


/**
 * Class that creates most of our hardware objects
 * Intercepts simulation and substitutes in a fake hardware device
 * Intercepts deviceNotFound and gives warning and a fake device
 */
@Config
public class HardwareCreator {
    public static boolean SIMULATE_HARDWARE = false;
    public static boolean SIMULATE_DRIVETRAIN = false;

    public enum ServoType {
        DEFAULT(600, 2400),
        GOBILDA(500, 2500),
        AXON(600, 2400); // technically can go 500-2500, but can end up with a continuous servo

        public final PwmControl.PwmRange pwmRange;

        ServoType(double usPulseLower, double usPulseUpper) {
            this.pwmRange = new PwmControl.PwmRange(usPulseLower, usPulseUpper);
        }
    }

    public static DcMotorEx createMotor(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new DcMotorFake();
        try {
            return hardwareMap.get(DcMotorEx.class, deviceName);
        } catch (IllegalArgumentException e) { // Could not find device
            RobotLog.addGlobalWarningMessage("Failed to find DcMotorEx '%s'", deviceName);
            return new DcMotorFake();
        }
    }

    public static DigitalChannel createDigitalChannel(HardwareMap hardwareMap, String deviceName) {
        if (SIMULATE_HARDWARE) return new DigitalChannelFake();
        try {
            return hardwareMap.digitalChannel.get(deviceName);
        } catch (IllegalArgumentException e) { // Could not find device
            RobotLog.addGlobalWarningMessage("Failed to find DigitalChannel '%s'", deviceName);
            return new DigitalChannelFake();
        }
    }

    public static Servo createServo(HardwareMap hardwareMap, String deviceName) {
        return createServo(hardwareMap, deviceName, ServoType.DEFAULT);
    }

    public static Servo createServo(HardwareMap hardwareMap, String deviceName, ServoType type) {
        if (SIMULATE_HARDWARE) return setServoRange(new ServoFake(), type);
        try {
            return setServoRange(hardwareMap.get(ServoImplEx.class, deviceName), type);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find Servo '%s'", deviceName);
            return setServoRange(new ServoFake(), type);
        }
    }

    public static CRServo createCRServo(HardwareMap hardwareMap, String deviceName) {
        return createCRServo(hardwareMap, deviceName, ServoType.DEFAULT);
    }

    public static CRServo createCRServo(HardwareMap hardwareMap, String deviceName, ServoType type) {
        if (SIMULATE_HARDWARE) return setServoRange(new CRServoFake(), type);
        try {
            return setServoRange(hardwareMap.get(CRServo.class, deviceName), type);
        } catch (IllegalArgumentException e) {
            RobotLog.addGlobalWarningMessage("Failed to find CRServo '%s'", deviceName);
            return setServoRange(new CRServoFake(), type);
        }
    }

    public static Servo setServoRange(Servo servo, ServoType servoType) {
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(servoType.pwmRange);
        }
        return servo;
    }

    public static CRServo setServoRange(CRServo crServo, ServoType servoType) {
        if (crServo instanceof PwmControl) {
            ((PwmControl) crServo).setPwmRange(servoType.pwmRange);
        }
        return crServo;
    }
}