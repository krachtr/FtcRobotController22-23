package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class T18555Navigation extends Thread{
    double fieldX = 0;
    double fieldY = 0;
    double fieldA = 0;

    double robotXErr;
    double robotYErr;
    double AErr;

    double robotXErr1 = 0;
    double robotYErr1 = 0;
    double AErr1 = 0;

    double robotXErr0;
    double robotYErr0;
    double AErr0;

    double Vx;
    double Vy;
    double Va;



    //get pos
    final int res = 28 * 20;

    double frontTicks0;
    double rightTicks0;
    double backTicks0;
    double leftTicks0;

    double frontTicks1;
    double rightTicks1;
    double backTicks1;
    double leftTicks1;

    double frontChangeInTicks;
    double rightChangeInTicks;
    double backChangeInTicks;
    double leftChangeInTicks;

    final double circumference = Math.PI * 90;

    final double mmPerTick = circumference/res;

    double changeInRobotX;
    double changeInRobotY;

    double changeInFieldX;
    double changeInFieldY;

    double changeInA;

    final double distanceFromCenterToWheel = 13.5 / 2 * 25.4 * 1.0861;

    //prop ditecter
    OpenCvWebcam webcam;

    boolean viewportPaused;

    boolean findZone = false;
    int zone = 0;
    Mat mat;

    T18555Navigation(HardwareMap hardwareMap){
        this.setDaemon(true);

        frontTicks1 = CrossDrive.front.getCurrentPosition();
        rightTicks1 = CrossDrive.right.getCurrentPosition();
        backTicks1 = CrossDrive.back.getCurrentPosition();
        leftTicks1 = CrossDrive.left.getCurrentPosition();

        initCam(hardwareMap);

        this.start();
    }
    @Override
    public void run() {
        while (true){
            getPos();
            calcErr(0,0,0);
        }
    }

    public void getPos(){
        //step 2
        frontTicks0 = frontTicks1;
        rightTicks0 = rightTicks1;
        backTicks0 = backTicks1;
        leftTicks0 = leftTicks1;

        frontTicks1 = CrossDrive.front.getCurrentPosition();
        rightTicks1 = CrossDrive.right.getCurrentPosition();
        backTicks1 = CrossDrive.back.getCurrentPosition();
        leftTicks1 = CrossDrive.left.getCurrentPosition();

        frontChangeInTicks = frontTicks1 - frontTicks0;
        rightChangeInTicks = rightTicks1 - rightTicks0;
        backChangeInTicks = backTicks1 - backTicks0;
        leftChangeInTicks = leftTicks1 - leftTicks0;

        double frontChangeInMM = frontChangeInTicks * mmPerTick;
        double rightChangeInMM = rightChangeInTicks * mmPerTick;
        double backChangeInMM = backChangeInTicks * mmPerTick;
        double leftChangeInMM = leftChangeInTicks * mmPerTick;

        changeInRobotX = (frontChangeInMM + backChangeInMM) / 2;
        changeInRobotY = (rightChangeInMM + leftChangeInMM) / 2;
        changeInA = (- frontChangeInMM - rightChangeInMM + backChangeInMM + leftChangeInMM) / 4 / distanceFromCenterToWheel;

        changeInFieldX = (changeInRobotX * Math.cos(fieldA + changeInA / 2)) + (-changeInRobotY * Math.sin(fieldA + changeInA / 2));
        changeInFieldY = (changeInRobotY * Math.cos(fieldA + changeInA / 2)) + (changeInRobotX * Math.sin(fieldA + changeInA / 2));

        fieldX += changeInFieldX;
        fieldY += changeInFieldY;
        fieldA += changeInA;
        if (fieldA >= 360){
            fieldA -= 360;
        }
    }
    public void calcErr(double X, double Y, double A){
        double fieldXErr = X - fieldX;
        double fieldYErr = Y - fieldY;
        AErr = A - fieldA;

        double directErr = Math.sqrt(Math.pow(fieldXErr, 2) + Math.pow(fieldYErr, 2));

        double directToXAngle = Math.atan(fieldXErr/fieldYErr);

        robotXErr = Math.cos(directToXAngle - fieldA) * directErr;
        robotYErr = Math.sin(directToXAngle - fieldA) * directErr;
    }

    public void celcXYAWithPID(double X, double Y, double A){
        final double KPx = .1;
        final double KPy = .1;
        final double KPa = .1;
        final double KIx = 0;
        final double KIy = 0;
        final double KIa = 0;
        final double KDx = 0;
        final double KDy = 0;
        final double KDa = 0;

        if (robotXErr1 != 0) {
            robotXErr0 = robotXErr1;
        } else {
            robotXErr0 = robotXErr;
        }
        robotXErr1 = robotXErr;

        Vx = KPx * (robotXErr) + KIx *  0 + KDx;
    }

    public void initCam(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new T18555Navigation.SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
    public int getTargetZone(int color){
        Rect roi1 = new Rect(65, 90, 70, 70);
        Rect roi2 = new Rect(135, 90, 70, 70);
        Rect roi3 = new Rect(205, 90, 70, 70);

        Mat cropped1 = new Mat(mat, roi1);
        Mat cropped2 = new Mat(mat, roi2);
        Mat cropped3 = new Mat(mat, roi3);

        MatOfDouble meansrc1 = new MatOfDouble();
        MatOfDouble stdsrc1 = new MatOfDouble();
        MatOfDouble meansrc2 = new MatOfDouble();
        MatOfDouble stdsrc2 = new MatOfDouble();
        MatOfDouble meansrc3 = new MatOfDouble();
        MatOfDouble stdsrc3 = new MatOfDouble();

        Core.meanStdDev(cropped1, meansrc1, stdsrc1);
        Core.meanStdDev(cropped2, meansrc2, stdsrc2);
        Core.meanStdDev(cropped3, meansrc3, stdsrc3);

        double lMeanSrc1 = meansrc1.get(0,0)[0];
        //double aMeanSrc1 = meansrc1.get(1,0)[0];
        double bMeanSrc1 = meansrc1.get(2,0)[0];
        double lMeanSrc2 = meansrc2.get(0,0)[0];
        //double aMeanSrc2 = meansrc2.get(1,0)[0];
        double bMeanSrc2 = meansrc2.get(2,0)[0];
        double lMeanSrc3 = meansrc3.get(0,0)[0];
        //double aMeanSrc3 = meansrc3.get(1,0)[0];
        double bMeanSrc3 = meansrc3.get(2,0)[0];

        //double lStdSrc = stdsrc1.get(0,0)[0];
        //double aStdSrc = stdsrc1.get(1,0)[0];
        //double bStdSrc = stdsrc1.get(2,0)[0];

        if(color == 0){
            //red
            if (lMeanSrc1 > lMeanSrc2 && lMeanSrc1 > lMeanSrc3){
                zone = 1;
            } else if (lMeanSrc2 > lMeanSrc3) {
                zone = 2;
            } else {
                zone = 3;
            }
        }else{
            if (bMeanSrc1 > bMeanSrc2 && bMeanSrc1 > bMeanSrc3){
                zone = 1;
            } else if (bMeanSrc2 > bMeanSrc3) {
                zone = 2;
            } else {
                zone = 3;
            }
        }
        return zone;
    }

    public double getFieldX() {
        return fieldX / 25.4;
    }
    public double getFieldY() {
        return fieldY / 25.4;
    }
    public double getFieldA() {
        return Math.toDegrees(fieldA);
    }
    public double getRobotXErr() {
        return robotXErr / 25.4;
    }
    public double getRobotYErr() {
        return robotYErr / 25.4;
    }
    public double getAErr() {
        return Math.toDegrees(AErr);
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            mat = input;
            Imgproc.rectangle(
                    input,
                    new Point(
                            65,
                            90),
                    new Point(
                            135,
                            160),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(
                    input,
                    new Point(
                            135,
                            90),
                    new Point(
                            205,
                            160),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(
                    input,
                    new Point(
                            205,
                            90),
                    new Point(
                            275,
                            160),
                    new Scalar(0, 255, 0), 4);


            return input;
        }

        @Override
        public void onViewportTapped()
        {

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }

}
