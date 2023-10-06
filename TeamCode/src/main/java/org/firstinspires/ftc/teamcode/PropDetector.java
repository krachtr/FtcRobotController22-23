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


public class PropDetector extends OpenCvPipeline{
    OpenCvWebcam webcam;

    boolean viewportPaused;

    boolean findZone = false;
    int zone = 0;
    Mat mat;

    PropDetector(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(this);
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
