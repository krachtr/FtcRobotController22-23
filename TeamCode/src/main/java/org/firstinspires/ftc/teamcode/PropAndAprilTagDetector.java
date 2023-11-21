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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;



public class PropAndAprilTagDetector extends OpenCvPipeline{
    OpenCvWebcam webcam;

    boolean viewportPaused;

    boolean findZone = false;
    int zone = 0;
    Mat mat;
    ArrayList<Double> returnThis = new ArrayList<>();

    //for April tag
    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    PropAndAprilTagDetector(HardwareMap hardwareMap){
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
        Rect roi1 = new Rect(40, 70, 30, 30);
        Rect roi2 = new Rect(165, 70, 30, 30);
        Rect roi3 = new Rect(280, 90, 30, 30);

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

        returnThis.clear();

        double lMeanSrc1 = meansrc1.get(0,0)[0];
        returnThis.add(lMeanSrc1);
        double aMeanSrc1 = meansrc1.get(1,0)[0];
        returnThis.add(aMeanSrc1);
        double bMeanSrc1 = meansrc1.get(2,0)[0];
        returnThis.add(bMeanSrc1);
        double lMeanSrc2 = meansrc2.get(0,0)[0];
        returnThis.add(lMeanSrc2);
        double aMeanSrc2 = meansrc2.get(1,0)[0];
        returnThis.add(aMeanSrc2);
        double bMeanSrc2 = meansrc2.get(2,0)[0];
        returnThis.add(bMeanSrc2);
        double lMeanSrc3 = meansrc3.get(0,0)[0];
        returnThis.add(lMeanSrc3);
        double aMeanSrc3 = meansrc3.get(1,0)[0];
        returnThis.add(aMeanSrc3);
        double bMeanSrc3 = meansrc3.get(2,0)[0];
        returnThis.add(bMeanSrc3);

        //double lStdSrc = stdsrc1.get(0,0)[0];
        //double aStdSrc = stdsrc1.get(1,0)[0];
        //double bStdSrc = stdsrc1.get(2,0)[0];

        if(color == 0){
            //red find the lowest blue
            if (bMeanSrc1 < bMeanSrc2 && bMeanSrc1 < bMeanSrc3){
                zone = 1;
            } else if (bMeanSrc2 < bMeanSrc3) {
                zone = 2;
            } else {
                zone = 3;
            }
        }else{
            //blue find the lowest red
            if (lMeanSrc1 < lMeanSrc2 && lMeanSrc1 < lMeanSrc3){
                zone = 1;
            } else if (lMeanSrc2 < lMeanSrc3) {
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
                        40,
                        70),
                new Point(
                        70,
                        100),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                input,
                new Point(
                        165,
                        70),
                new Point(
                        195,
                        100),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                input,
                new Point(
                        280,
                        90),
                new Point(
                        310,
                        120),
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
    public ArrayList<Double> getData(){
        return returnThis;
    }
}
