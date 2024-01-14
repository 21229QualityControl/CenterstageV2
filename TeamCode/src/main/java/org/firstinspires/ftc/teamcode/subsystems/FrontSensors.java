package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@Config
public class FrontSensors {
    public static Queue<Integer> values = new LinkedList<>();
    public static int SAMPLE_COUNT = 10;
    private DistanceSensor leftSensor;
    private DistanceSensor middleSensor;
    private DistanceSensor rightSensor;

    // April tags
    private AprilTagProcessor processor;
    private VisionPortal portal;

    public FrontSensors(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        middleSensor = hardwareMap.get(DistanceSensor.class, "middleDistanceSensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
//        processor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // TODO: Calibrate for real camera, this is for logitech c720
//                .build();
//        portal = new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
//                .setCameraResolution(new Size(640, 480))
//                .build();
    }

//    public List<AprilTagDetection> detectTags() {
//        return processor.getDetections();
//    }
//
//    int frameCount = -1;
//    double lastValue = 0;
//    public double backdropDistance(int refreshFrames) {
//        if (frameCount > 0 && frameCount < refreshFrames) {
//            return lastValue;
//        }
//        List<AprilTagDetection> det = detectTags();
//        double lowestDistance = Double.MAX_VALUE;
//        for (AprilTagDetection d : det) {
//            if (d.metadata != null) {
//                if (d.ftcPose.range < lowestDistance) {
//                    lowestDistance = d.ftcPose.range;
//                }
//            }
//        }
//        lastValue = lowestDistance;
//        return lowestDistance;
//    }

    public int rawObjectPosition() {
        double lDistance = leftSensor.getDistance(DistanceUnit.INCH);
        double mDistance = middleSensor.getDistance(DistanceUnit.INCH);
        double rDistance = rightSensor.getDistance(DistanceUnit.INCH);

        if (lDistance > 300 & mDistance > 300 & rDistance > 300) {
            return -1; // Default position
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
