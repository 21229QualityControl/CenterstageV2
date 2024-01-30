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
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;
@Config
public class PixelStackProcessor implements VisionProcessor, CameraStreamSource {

    public static int HsvSensitivity = 40;
    public static int BoundingWidth = 100;
    public static int BoundingHeight = 160;
    public static Rect Stack = new Rect((int) (320 - Math.round(0.5 * BoundingWidth)),
            480 - BoundingHeight, BoundingWidth , BoundingHeight);
    public static int SlidingWidth = 60;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final AtomicReference<Bitmap> lastFrameWithBoundingRect =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private CameraUtil.DebugMode debugMode = CameraUtil.DebugMode.None;

    Mat hsvMat = new Mat();
    private int width = 640;
    private int height = 480;

    private double sumWhiteScalar = 0;
    private int sumWhites = 0;
    private double percentageWhites = 0.0;

    private boolean detected = false;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Rect targetRect = detectWhiteStack(frame);
        saveDebugMat(frame, targetRect);
        detected = targetRect == null? false: true;
        return detected;
    }

    public boolean getDetected() {
        return detected;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrameWithBoundingRect.get()));
        }
    }

    public void setDebugMode(CameraUtil.DebugMode debugMode) {
        this.debugMode = debugMode;
    }

    private Rect detectWhiteStack(Mat frame) {
        // The white pixel stack would appear (on an HSV image)
        // as a region of high V, and very low S. H can be anything.
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerWhite = new Scalar(0, 0, 255 - HsvSensitivity);
        Scalar upperWhite = new Scalar(255, HsvSensitivity, 255);

        Mat whiteMask = new Mat();
        Core.inRange(hsvMat, lowerWhite, upperWhite, whiteMask);

        Rect target = getWhiteStackPos(whiteMask);
        return target;
    }

    private Rect getWhiteStackPos(Mat frame) {
        int x = 0;
        int y = 480 - BoundingHeight;
        //Scalar max = new Scalar(0);
        int max = 0;
        Rect maxRect = null;
        while (x < width) {
            Rect rect = getSlidingBoundingBox(x, y, BoundingWidth, BoundingHeight);
            if (rect == null) {
                break;
            }
            Mat subMat = frame.submat(rect);
            Double area = (double) (subMat.width() * subMat.height());
            //Scalar totalWhites = Core.sumElems(subMat);
            int totalWhites = Core.countNonZero(subMat);
            // if (totalWhites.val[0] > max.val[0]) {
            if (totalWhites > max) {
                max = totalWhites;
                maxRect = rect;
                // sumWhiteScalar = totalWhites.val[0];
                sumWhites = totalWhites;
                percentageWhites = (double) sumWhites / area;
            }

            x += SlidingWidth;
        }

        return maxRect;
    }

    private Rect getSlidingBoundingBox(int x, int y, int width, int height) {
        if (x >= this.width) {
            return null;
        }

        if (y >= this.height) {
            return null;
        }

        int good_width = Math.min(width, this.width - x);
        int good_height = Math.min(height, this.height - y);
        return new Rect(x, y, good_width, good_height);
    }

    private void saveDebugMat(Mat frame, Rect rect) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            // Convert the frame to the Bitmap "b."
            Utils.matToBitmap(frame, b);
            // Set the "lastFrame" to the Bitmap "b." So that AtomicReference container ACTUALLY holds the Bitmap.
            lastFrame.set(b);

            if (rect != null) {
                // Make a color "green."
                Scalar green = new Scalar(0, 255, 0);
                // Clone the Matrix. The original matrix is for the processing purposes and the cloned matrix is for the display purposes.
                Mat frameWithBoundingRect = frame.clone();
                // Draw rectangles on the cloned matrix.
                Imgproc.rectangle(frameWithBoundingRect, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), green, 4);

                // Create a Bitmap "bb" that is the same size as "frameWithBoundingRect."
                Bitmap bb = Bitmap.createBitmap(frameWithBoundingRect.width(), frameWithBoundingRect.height(), Bitmap.Config.RGB_565);
                // Convert the cloned matrix to the Bitmap "bb."
                Utils.matToBitmap(frameWithBoundingRect, bb);
                // Set the "lastFrameWithBoundingRect" to the Bitmap "bb." So that AtomicReference container ACTUALLY holds the Bitmap.
                lastFrameWithBoundingRect.set(bb);
            }
        }
    }
}
