package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Canvas;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

// https://github.com/KookyBotz/CenterStage/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/common/vision/PreloadDetectionPipeline.java#L107

public class PartnerPreloadProcessor implements VisionProcessor {
    private AprilTagProcessor aprilTag;
    int targetAprilTag;
    public boolean preloadLeft = false;
    public int leftZoneAverage;
    public int rightZoneAverage;
    public boolean detecting = false;
    public boolean fallback = false;

    public static int fallbackCenterX = 0;
    public static int fallbackCenterY = 0;
    public static int fallbackWidth = 0;
    public static int fallbackHeight = 0;

    public PartnerPreloadProcessor(AprilTagProcessor aprilTag) {
        this.aprilTag = aprilTag;
    }
    public void updateTarget(int spike, boolean red) {
        targetAprilTag = (3 - spike); // 0 = right, 1 = middle, 2 = left
        if (red) {
            targetAprilTag += 3;
        }
    }

    public static AprilTagProcessor newAprilTagProcessor() {
        return new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // TODO: Calibrate for real camera, this is for logitech c720
                .build();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        List<AprilTagDetection> det = aprilTag.getDetections();
        if (det != null && !fallback) {
            return null;
        }

        int tagCenterX = 0;
        int tagCenterY = 0;
        int tagWidth = 0;
        int tagHeight = 0;

        boolean foundTag = false;
        for (AprilTagDetection d : det) {
            if (d.id != targetAprilTag) {
                continue;
            }

            // General logic taken from kookybotz
            int leftX = Integer.MAX_VALUE;
            int rightX = Integer.MIN_VALUE;
            int topY = Integer.MIN_VALUE;
            int bottomY = Integer.MAX_VALUE;

            for (Point point : d.corners) {
                if (point.x < leftX) leftX = (int) point.x;
                if (point.x > rightX) rightX = (int) point.x;
                if (point.y > topY) topY = (int) point.y;
                if (point.y < bottomY) bottomY = (int) point.y;
            }

            tagCenterX = (int) d.center.x;
            tagCenterY = (int) d.center.y;

            tagWidth = rightX - leftX;
            tagHeight = topY - bottomY;

            foundTag = true; // Found it!
        }
        if (!foundTag) {
            if (fallback) {
                tagCenterX = fallbackCenterX;
                tagCenterY = fallbackCenterY;
                tagWidth = fallbackWidth;
                tagHeight = fallbackHeight;
            } else {
                return null;
            }
        }

        int inclusionZoneWidth = (int) (tagWidth * 1.3);
        int inclusionZoneHeight = (int) (tagHeight * 1.3);

        int exclusionZoneWidth = (int) (tagWidth * 0.28);
        int exclusionZoneHeight = (int) (tagHeight * 0.28);

        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - (int)(inclusionZoneHeight*1.8), inclusionZoneWidth, inclusionZoneHeight);
        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - (int)(inclusionZoneHeight*1.8), inclusionZoneWidth, inclusionZoneHeight);

        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.6), tagCenterY - (int)(inclusionZoneHeight*1.3), exclusionZoneWidth, exclusionZoneHeight);
        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.35), tagCenterY - (int)(inclusionZoneHeight*1.3), exclusionZoneWidth, exclusionZoneHeight);

        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);

        Imgproc.rectangle(frame, leftExclusionZone, new Scalar(255, 0, 0), 7);
        Imgproc.rectangle(frame, rightExclusionZone, new Scalar(255, 0, 0), 7);

        leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
        rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

        preloadLeft = leftZoneAverage > rightZoneAverage;
        detecting = true;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    // Thanks KookyBotz
    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        if (frame == null) {
            Log.d("PartnerPreloadProcessor", "Frame is null");
            return 0;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }

                if (x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= exclusionRect.y && y < exclusionRect.y + exclusionRect.height) {
                    continue;
                }

                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        return count > 0 ? sum / count : 0;
    }
}
