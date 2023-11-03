package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class Vision {
    public static Queue<Integer> values = new LinkedList<>();
    public static int SAMPLE_COUNT = 10;
    private DistanceSensor leftSensor;
    private DistanceSensor middleSensor;
    private DistanceSensor rightSensor;

    public Vision(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        middleSensor = hardwareMap.get(DistanceSensor.class, "middleDistanceSensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

    public int rawObjectPosition() {
        double lDistance = leftSensor.getDistance(DistanceUnit.INCH);
        double mDistance = middleSensor.getDistance(DistanceUnit.INCH);
        double rDistance = rightSensor.getDistance(DistanceUnit.INCH);

        if (lDistance > 300 & mDistance > 300 & rDistance > 300) {
            return 0; // Default position
        }
        else if (lDistance > mDistance) {
            if (mDistance > rDistance) {
                return 2;
            }
            else {
                return 1;
            }
        }
        else if (lDistance > rDistance) {
            return 2;
        }
        else {
            return 0;
        }
    }

    // 0 = left
    // 1 = middle
    // 2 = right
    public int objectPosition() {
        int pos = rawObjectPosition();
        values.add(pos);
        if (values.size() > SAMPLE_COUNT) {
            values.remove();
        }
        double total = 0;
        for (int val : values) {
            total += val;
        }
        return (int) Math.round(total / SAMPLE_COUNT);
    }

    public void displayTelemetry(Telemetry telemetry) {
        double lDistance = leftSensor.getDistance(DistanceUnit.INCH);
        double mDistance = middleSensor.getDistance(DistanceUnit.INCH);
        double rDistance = rightSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("l sensor", lDistance);
        telemetry.addData("m sensor", mDistance);
        telemetry.addData("r sensor", rDistance);
    }
}
