package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.HardwareCreator;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

public class Slider {
    public static PIDCoefficients outtakePID = new PIDCoefficients(0.004, 0.0015, 0.0004);

    private DcMotorEx motor;


    public Slider(HardwareMap hardwareMap) {
        this.motor = HardwareCreator.createMotor(hardwareMap, "outtakeSlide");
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void prepTeleop() {
        this.motor.setPower(-0.3);
    }
}
