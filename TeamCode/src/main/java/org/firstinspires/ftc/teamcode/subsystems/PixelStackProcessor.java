package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.CameraUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
@Config
public class PixelStackProcessor implements VisionProcessor, CameraStreamSource {

    public static int HsvSensitivity = 40;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private CameraUtil.DebugMode debugMode = CameraUtil.DebugMode.None;

    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        hsvDetection(frame);

        /*
        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(whiteRange, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours
        Mat result = new Mat();
        Imgproc.drawContours(result, contours, -1, new Scalar(0, 255, 0), 3);

        if (debugMode == DebugMode.Dashboard) {
            Bitmap bb = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565);
            Utils.matToBitmap(result, bb);
            whiteContourFrame.set(bb);
        }
         */
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    public void setDebugMode(CameraUtil.DebugMode debugMode) {
        this.debugMode = debugMode;
    }

    private void hsvDetection(Mat frame) {
        // The white pixel stack would appear (on an HSV image)
        // as a region of high V, and very low S. H can be anything.
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerWhite = new Scalar(0, 0, 255 - HsvSensitivity);
        Scalar upperWhite = new Scalar(255, HsvSensitivity, 255);

        Mat whiteRange = new Mat();
        Core.inRange(hsvMat, lowerWhite, upperWhite, whiteRange);
        saveDebugMat(whiteRange);
    }

    private void saveDebugMat(Mat frame) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
        }
    }
}
