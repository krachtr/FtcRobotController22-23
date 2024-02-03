package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class PropDetect implements VisionProcessor {
//  reference EOCV-Sim:
//  https://deltacv.gitbook.io/eocv-sim/vision-portal/introduction-to-visionportal/creating-and-running-a-visionprocessor
    Mat matForProp;



    public static Point leftBox = new Point(0,193);
    public static Point centerBox = new Point(592,209);
    public static Point rightBox = new Point(1104,354);
    public static int width = 112;
    public static int height = 145;


    @Override
    public void init(int width, int height, CameraCalibration calibration){
        // Initialization code here
        // Executed before the first call to processFrame
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        // Process the frame here
        // Executed every time a new frame is dispatched
        // if you change the mat "frame", that change will display

        matForProp = frame;

        return frame;// Return the image that will be displayed in the viewport
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.left * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.top * scaleBmpPxToCanvasPx);
        int right = Math.round(rect.right * scaleBmpPxToCanvasPx);
        int bottom =Math.round(rect.bottom * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // Create a Paint object to set drawing attributes
        Paint paint = new Paint();
        paint.setColor(Color.RED); // Set the color to red
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(8);

        // Draw a red rectangle on the Canvas
        Rect leftRect = new Rect(leftBox.x, leftBox.y, leftBox.x + width,leftBox.y + height);
        canvas.drawRect(makeGraphicsRect(leftRect, scaleBmpPxToCanvasPx), paint);

        Rect centerRect = new Rect(centerBox.x, centerBox.y, centerBox.x + width,centerBox.y + height);
        canvas.drawRect(makeGraphicsRect(centerRect, scaleBmpPxToCanvasPx), paint);

        Rect rightRect = new Rect(rightBox.x, rightBox.y, rightBox.x + width,rightBox.y + height);
        canvas.drawRect(makeGraphicsRect(rightRect, scaleBmpPxToCanvasPx), paint);
    }

    public int getTargetZone(int color){
        int zone = 0;

        org.opencv.core.Rect left = new org.opencv.core.Rect(leftBox.x, leftBox.y, width, height);
        org.opencv.core.Rect center = new org.opencv.core.Rect(centerBox.x, centerBox.y, width, height);
        org.opencv.core.Rect right = new org.opencv.core.Rect(rightBox.x, rightBox.y, width, height);

        if (matForProp !=null) {
            Mat croppedLeft = new Mat(matForProp, left);
            Mat croppedCenter = new Mat(matForProp, center);
            Mat croppedRight = new Mat(matForProp, right);

            MatOfDouble meanLeft = new MatOfDouble();
            MatOfDouble stdLeft = new MatOfDouble();
            MatOfDouble meanCenter = new MatOfDouble();
            MatOfDouble stdCenter = new MatOfDouble();
            MatOfDouble meanRight = new MatOfDouble();
            MatOfDouble stdRight = new MatOfDouble();

            Core.meanStdDev(croppedLeft, meanLeft, stdLeft);
            Core.meanStdDev(croppedCenter, meanCenter, stdCenter);
            Core.meanStdDev(croppedRight, meanRight, stdRight);

//            returnThis.clear();
//
            double lMeanSrc1 = meanLeft.get(0,0)[0];
//            returnThis.add(lMeanSrc1);
//            double aMeanSrc1 = meanLeft.get(1,0)[0];
//            returnThis.add(aMeanSrc1);
            double bMeanSrc1 = meanLeft.get(2,0)[0];
//            returnThis.add(bMeanSrc1);
            double lMeanSrc2 = meanCenter.get(0,0)[0];
//            returnThis.add(lMeanSrc2);
//            double aMeanSrc2 = meanCenter.get(1,0)[0];
//            returnThis.add(aMeanSrc2);
            double bMeanSrc2 = meanCenter.get(2,0)[0];
//            returnThis.add(bMeanSrc2);
            double lMeanSrc3 = meanRight.get(0,0)[0];
//            returnThis.add(lMeanSrc3);
//            double aMeanSrc3 = meanRight.get(1,0)[0];
//            returnThis.add(aMeanSrc3);
            double bMeanSrc3 = meanRight.get(2,0)[0];
//            returnThis.add(bMeanSrc3);


            if (color == 0) {
                //red find the lowest blue
                if (bMeanSrc1 < bMeanSrc2 && bMeanSrc1 < bMeanSrc3) {
                    zone = 1;
                } else if (bMeanSrc2 < bMeanSrc3) {
                    zone = 2;
                } else {
                    zone = 3;
                }
            } else {
                //blue find the lowest red
                if (lMeanSrc1 < lMeanSrc2 && lMeanSrc1 < lMeanSrc3) {
                    zone = 1;
                } else if (lMeanSrc2 < lMeanSrc3) {
                    zone = 2;
                } else {
                    zone = 3;
                }
            }
            return zone;
        } else {
            return 4;
        }
    }
//    public class SamplePipeline extends OpenCvPipeline {
//          https://deltacv.gitbook.io/eocv-sim/introduction/pipelines/how-it-works
//
//        @Override
//        public void init(Mat input) {
//            // Executed before the first call to processFrame
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            // Executed every time a new frame is dispatched
//
//            return input; // Return the image that will be displayed in the viewport
//            // (In this case the input mat directly)
//        }
//
//        @Override
//        public void onViewportTapped() {
//            // Executed when the image display is clicked by the mouse or tapped
//            // This method is executed from the UI thread, so be careful to not
//            // perform any sort heavy processing here! Your app might hang otherwise
//        }
//
//    }


}
