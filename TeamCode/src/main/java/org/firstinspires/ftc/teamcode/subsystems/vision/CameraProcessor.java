package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public class CameraProcessor implements VisionProcessor, CameraStreamSource {
    // State the dimensions of the rectangles for the 3 locations of the team prop.
    public static Rect leftRect = new Rect(100, 350, 116, 93);
    public static Rect centerRect = new Rect(245, 330, 180, 70);
    public static Rect rightRect = new Rect(450, 350, 116, 93);
    public int position = -1;
    private double satRectLeft;
    private double satRectCenter;
    private double satRectRight;
    Mat hsvMat = new Mat();

    private CameraUtil.DebugMode debugMode = CameraUtil.DebugMode.Dashboard;

    /* Create an AtomicReference that holds a Bitmap that is called "lastFrame" and give it some random dimensions.
     * The "AtomicReference" means that it is thread safe.
     */
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /* Create an AtomicReference that holds a Bitmap that's called "lastFrameWithBoundingRect" and give it some random dimensions.
     * This Bitmap is meant for rectangles to be drawn on it for debugging purposes.
     * But we don't want to actually use this to get the HSV values since the rectangles could skew the values.
     */
    private AtomicReference<Bitmap> lastFrameWithBoundingRect =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public void init(int width, int height, CameraCalibration calibration) {
        // Do nothing. Since I implemented some classes in this class I need to implement their methods. Unfortunately I don't need them.
    };

    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            saveDebugMat(frame);
        }

        //-----
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satRectLeft = getAvgSaturation(hsvMat, leftRect);
        satRectCenter = getAvgSaturation(hsvMat, centerRect);
        satRectRight = getAvgSaturation(hsvMat, rightRect);

        if (satRectLeft > satRectCenter && satRectLeft > satRectRight) {
            // position = "left";
            position = 2;
        } else if (satRectCenter > satRectRight) {
            // position = "center";
            position = 1;
        } else {
            // position = "right";
            position = 0;
        }

        return position;
    };

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (debugMode == CameraUtil.DebugMode.DriverStation) {
            Paint paint=new Paint();
            paint.setColor(Color.GREEN);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(scaleCanvasDensity * 4);

            android.graphics.Rect left=makeGraphicsRect(leftRect, scaleBmpPxToCanvasPx);
            android.graphics.Rect center=makeGraphicsRect(centerRect, scaleBmpPxToCanvasPx);
            android.graphics.Rect right=makeGraphicsRect(rightRect, scaleBmpPxToCanvasPx);

            canvas.drawRect(left, paint);
            canvas.drawRect(center, paint);
            canvas.drawRect(right, paint);
        }
    };

    // This lets Dashboard get all of our Bitmaps!!!
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (debugMode == CameraUtil.DebugMode.Dashboard) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrameWithBoundingRect.get()));
        }
    }

    public double getSatRectLeft() {
        return satRectLeft;
    }

    public double getSatRectCenter() {
        return satRectCenter;
    }

    public double getSatRectRight() {
        return satRectRight;
    }

    public void setDebugMode(CameraUtil.DebugMode debugMode) {
        this.debugMode = debugMode;
    }

    private double getAvgSaturation(Mat input, Rect rect) {
        Mat subMat = input.submat(rect);
        return Core.mean(subMat).val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left=Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top=Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right=left+Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom=top+Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left,top,right,bottom);
    }

    private void saveDebugMat(Mat frame) {
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        // Convert the frame to the Bitmap "b."
        Utils.matToBitmap(frame, b);
        // Set the "lastFrame" to the Bitmap "b." So that AtomicReference container ACTUALLY holds the Bitmap.
        lastFrame.set(b);
        // Make a color "green."
        Scalar green = new Scalar(0, 255, 0);
        // Clone the Matrix. The original matrix is for the processing purposes and the cloned matrix is for the display purposes.
        Mat frameWithBoundingRect = frame.clone();
        // Draw rectangles on the cloned matrix.
        Imgproc.rectangle(frameWithBoundingRect, new Point(leftRect.x, leftRect.y), new Point(leftRect.x + leftRect.width, leftRect.y + leftRect.height), green, 4);
        Imgproc.rectangle(frameWithBoundingRect, new Point(centerRect.x, centerRect.y), new Point(centerRect.x + centerRect.width, centerRect.y + centerRect.height), green, 4);
        Imgproc.rectangle(frameWithBoundingRect, new Point(rightRect.x, rightRect.y), new Point(rightRect.x + rightRect.width, rightRect.y + rightRect.height), green, 4);
        // Create a Bitmap "bb" that is the same size as "frameWithBoundingRect."
        Bitmap bb = Bitmap.createBitmap(frameWithBoundingRect.width(), frameWithBoundingRect.height(), Bitmap.Config.RGB_565);
        // Convert the cloned matrix to the Bitmap "bb."
        Utils.matToBitmap(frameWithBoundingRect, bb);
        // Set the "lastFrameWithBoundingRect" to the Bitmap "bb." So that AtomicReference container ACTUALLY holds the Bitmap.
        lastFrameWithBoundingRect.set(bb);
    }
}
