/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PropDetect;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

/**
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 * <p>
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 * <p>
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 * <p>
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 * <p>
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 * <p>
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 * <p>
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="Auto State", group = "A")

public class AutoState extends LinearOpMode

{
    TrajectorySequence z1_1_p1;
    double lineupY = 0;
    TrajectorySequence lineupMove;
    double target = 0;
    double ldis = 0;
    double rdis = 0;

    double Ltarget = 2.7;

    double Rtarget = 2;

    private DistanceSensor sensorDistanceL;
    private DistanceSensor sensorDistanceR;

    // Adjust these numbers to suit your robot.
//    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
//    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backDrive = null;  //  Used to control the right back drive wheel

    private DcMotor inMotor = null;
    private DcMotor outMotor = null;
    private DcMotor graberMotor = null;
    private DcMotorEx upMotor = null;

    public Servo shooter;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    // private static final int DESIRED_TAG_ID = 10;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.

    //TODO: move to init camera function????
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private PropDetect prop;
    // private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //for cam
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int ID_TAG_OF_INTEREST = 7; // Tag ID 18 from the 36h11 family
//
//    org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

    long wate1 = 0;
    long wate2 = 0;

    boolean farePath = false;

    boolean parkFar = false;

    private ElapsedTime     runtime = new ElapsedTime();

    SampleMecanumDrive drive;






    @Override public void runOpMode() throws InterruptedException {

        // initialize the camera
        initCameraProcessor();

//This is initializing The intake servos.
        int grabUpPos = 26594;
        int grabHangPos = 15661;
        int grabCamPos = 3395;
        int grabDownPos = 0;

        int outLoadPos = 0;
        int outDropPos = 580;
        int outLineupPos = -250;

        int upRedyPos = 419;
        int upStartPos = 670;
        inMotor = hardwareMap.get(DcMotor.class, "in");
        outMotor = hardwareMap.get(DcMotor.class, "out");
        graberMotor = hardwareMap.get(DcMotor.class, "grab");
        upMotor = hardwareMap.get(DcMotorEx.class, "up");

        inMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        graberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.get(Servo.class, "shooter");

        inMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outMotor.setTargetPosition(outLoadPos);
        outMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        upMotor.setTargetPosition(upStartPos);
        upMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        graberMotor.setTargetPosition(grabCamPos);
        graberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//This is initializing The arm motor.

        int program = 1;
        telemetry.addData("sulect a starting position whith the dPad: ", "blue left");
        telemetry.update();
        while (!gamepad1.a){
            if(gamepad1.dpad_up | gamepad1.dpad_down) {
                if(gamepad1.dpad_up) {
                    program++;
                } else {
                    program --;
                }

                if (program < 1){
                    program = 4;
                } else if (program > 4) {
                    program = 1;
                }
                if (program == 1) {
                    telemetry.addData("sulect a starting position whith the dPad: ", "blue left");
                } else if (program == 2) {
                    telemetry.addData("sulect a starting position whith the dPad: ", "blue right");
                } else if (program == 3) {
                    telemetry.addData("sulect a starting position whith the dPad: ", "red left");
                } else {
                    telemetry.addData("sulect a starting position whith the dPad: ", "red right");
                }
                telemetry.update();

                while (gamepad1.dpad_up | gamepad1.dpad_down){
                    Thread.sleep(10);
                }


//            } else if (gamepad1.dpad_down) {
//                program --;
//                if (program < 1){
//                    program = 4;
//                } else if (program > 4) {
//                    program = 1;
//                }
//                if (program == 1) {
//                    telemetry.addData("select a starting position with the dPad: ", "blue left");
//                } else if (program == 2) {
//                    telemetry.addData("select a starting position with the dPad: ", "blue right");
//                } else if (program == 3) {
//                    telemetry.addData("select a starting position with the dPad: ", "red left");
//                } else {
//                    telemetry.addData("select a starting position with the dPad: ", "red right");
//                }
//                telemetry.update();
//                while (gamepad1.dpad_down){
//                    Thread.sleep(10);
//                }
            }

            Thread.sleep(10);
        }
//This tells the robot where on the field it is starting.
        while (gamepad1.a){
            Thread.sleep(10);
        }
        while (!gamepad1.a){
            if(gamepad1.dpad_up){
                wate1 += 200;
            } else if (gamepad1.dpad_down) {
                wate1 -= 200;
            }
            if (wate1 < 0){
                wate1 = 0;
            } else if (wate1 > 30000) {
                wate1 = 30000;
            }
            telemetry.addData("1st wait until:", wate1);
            telemetry.update();
            Thread.sleep(50);
        }
        while (gamepad1.a){
            Thread.sleep(10);
        }
        while (!gamepad1.a){
            if(gamepad1.dpad_up){
                wate2 += 200;
            } else if (gamepad1.dpad_down) {
                wate2 -= 200;
            }
            if (wate2 < 0){
                wate2 = 0;
            } else if (wate2 > 30000) {
                wate2 = 30000;
            }
            telemetry.addData("2nd wait until:", wate2);
            telemetry.update();
            Thread.sleep(50);
        }
        while (gamepad1.a){
            Thread.sleep(10);
        }
        while (!gamepad1.a){
            if(gamepad1.dpad_up){
                farePath = !farePath;
                while (gamepad1.dpad_up){
                    Thread.sleep(10);
                }
            } else if (gamepad1.dpad_down) {
                farePath = !farePath;
                while (gamepad1.dpad_down){
                    Thread.sleep(10);
                }
            }
            telemetry.addData("Far Path = ", farePath);
            telemetry.update();
            Thread.sleep(50);
        }
        while (gamepad1.a){
            Thread.sleep(10);
        }
        while (!gamepad1.a){
            if(gamepad1.dpad_up){
                parkFar = !parkFar;
                while (gamepad1.dpad_up){
                    Thread.sleep(10);
                }
            } else if (gamepad1.dpad_down) {
                parkFar = !parkFar;
                while (gamepad1.dpad_down){
                    Thread.sleep(10);
                }
            }
            telemetry.addData("Park Far = ", parkFar);
            telemetry.update();
            Thread.sleep(50);
        }
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  driveForword    = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        int zone = 1;





        drive = new SampleMecanumDrive(hardwareMap);

// This is all the possible places that the robot could start.
        if (program == 1) {
            // blue left, closest to back bord
            difinePaths(2);
        } else if (program == 2) {
            // blue right, farthest to back bord
            difinePaths(4);
        } else if (program == 3) {
            // red left, farthest to back bord
            difinePaths(3);
        } else {
            // red right, closest to back bord
            difinePaths(1);
        }

//This is the different paths the robot will take depending on where it is.
        /*// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        frontDrive = hardwareMap.get(DcMotor.class, "front");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        backDrive.setDirection(DcMotor.Direction.REVERSE);*/


        //if (USE_WEBCAM)
            //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        //PropDetector propDetector = new PropDetector(hardwareMap);
        if (program == 1) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "blue left program selected");
        } else if (program == 2) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "blue right program selected");
        } else if (program == 3) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "red left program selected");
        } else {
            //drive.setPoseEstimate(new Pose2d(61, 9, Math.toRadians(135)));
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "red right program selected");
        }
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()){
            if (program == 1 || program == 2) {
                //Blue
                PoseStorage.redSide = false;
                zone = prop.getTargetZone(1); //1;//aprilTagDetectionPipeline.getTargetZone(1);
            } else {
                //Red
                zone = prop.getTargetZone(0); //1;//aprilTagDetectionPipeline.getTargetZone(0);
                PoseStorage.redSide = true;
            }
            telemetry.addData("prop detected in zone:", zone);
            telemetry.update();
            Thread.sleep(50);
        }

//This shows what program we have selected.
// for cam
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //camera.setPipeline(aprilTagDetectionPipeline);
        //{
        //    @Override
        //    public void onOpened()
         //   {
          //      camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
          //  }

          //  @Override
        //    public void onError(int errorCode)
         //   {

        //    }
        //});

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//This is initializing the camera.
        /*while (!isStarted() && !isStopRequested())
        {
            ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//This is the list of April tags.
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
//This says which April tag it see.
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
//This tell that it doesn't see any of the April tags.
            }
            if (program == 1 || program == 2) {
                //Blue
                telemetry.addData("Zone", aprilTagDetectionPipeline.getTargetZone(1));
            } else {
                //Red
                telemetry.addData("Zone", aprilTagDetectionPipeline.getTargetZone(0));
            }
            telemetry.update();
            sleep(20);
        }
//This says where the camera see the prop.
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        /*if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
 //This is if the camera picks one of the tags.
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
//The is if the camera didn't pickup any of the tags.
        /* Actually do something useful */
        /*if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
       /* }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
        /*    if(tagOfInterest.pose.x <= 20)
            {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                // do something else
            }
        }*/


        // Initialize the Apriltag Detection process

        drive.followTrajectorySequenceAsync(z1_1_p1);
        waitForStart();
        runtime.reset();

//        if (program == 1 || program == 2) {
//            //Blue
//            PoseStorage.redSide = false;
//            zone = prop.getTargetZone(1); //1;//aprilTagDetectionPipeline.getTargetZone(1);
//        } else {
//            //Red
//            zone = prop.getTargetZone(0); //1;//aprilTagDetectionPipeline.getTargetZone(0);
//            PoseStorage.redSide = true;
//        }

        //shooter.setPosition(.75);
        ((DcMotorEx) graberMotor).setVelocity(10000);
        outMotor.setPower(.45);
        //upMotor.setPower(.2);
        graberMotor.setTargetPosition(grabDownPos);
//This tells the robot to look for the red or the blue prop.


//This is for the movement of the robot on the red side.

        double i = 0;
        while (opModeIsActive())
        {

            PoseStorage.currentPose = drive.getPoseEstimate();
            // Step through the list of detected tags and look for a matching tag
            telemetry.addData("pos",drive.getPoseEstimate());

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("d",currentDetections);

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    Vector<Double> output = getCammraPosWithAprilTag(detection);
                    telemetry.addData("theta", output.get(0));
                    telemetry.addData("x", output.get(1));
                    telemetry.addData("y", output.get(2));
                    output = getRobotPosFromCameraStream(output.get(0), output.get(2), output.get(1));
                    //drive.setPoseEstimate(new Pose2d(output.get(1),output.get(2),output.get(0)));
                }
            }
            telemetry.update();
            //drive.update();

        }

    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double Front    =  y +yaw;
        double Right   =  -x +yaw;
        double Left     =  -x -yaw;
        double Back    =  y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(Front), Math.abs(Back));
        max = Math.max(max, Math.abs(Left));
        max = Math.max(max, Math.abs(Right));

        if (max > 1.0) {
            Front /= max;
            Right /= max;
            Left /= max;
            Back /= max;
        }

        // Send powers to the wheels.
        frontDrive.setPower(Front);
        rightDrive.setPower(Right);
        leftDrive.setPower(Left);
        backDrive.setPower(Back);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initCameraProcessor() {

        AprilTagProcessor myAprilTagProcessor;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        /* Optional: custom april tag library:
           AprilTagLibrary.Builder myAprilTagLibraryBuilder
           AprilTagLibrary myAprilTagLibrary;

        // Create a new AprilTagLibrary.Builder object and assigns it to a variable.
           myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Get the AprilTagLibrary for the current season.
           myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add a tag, without pose information, to the AprilTagLibrary.Builder.
           myAprilTagLibraryBuilder.addTag(10, "Our Awesome Team Tag", 5, DistanceUnit.INCH);

        // Build the AprilTag library and assign it to a variable.
           myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create a new AprilTagProcessor.Builder object
           myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Set the tag library.
           myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
           aprilTag = myAprilTagProcessorBuilder.build();

         */

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // reads 73.3, should be 48 .setLensIntrinsics(1439., 1439., 658.990, 350.764) cx/fx = .458 cy/fy=.244
                //.setLensIntrinsics(879.145, 879.145, 297.525, 260.564)
                .setLensIntrinsics(1414.88,1414.88,709.08,352.476)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        prop = new PropDetect();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(prop);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    void tagToTelemetry(org.openftc.apriltag.AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

    public void lineUpWithSensors(SampleMecanumDrive drive,double target2, double untilErr) {
        target = target2;
        ldis = sensorDistanceL.getDistance(DistanceUnit.INCH);
        rdis = sensorDistanceR.getDistance(DistanceUnit.INCH);
        //target - ((ldis - Ltarget)+ (rdis - Rtarget))/2
        while (((ldis - Ltarget)+ (rdis - Rtarget))/2 > untilErr){
            lineupY = target - ((ldis - Ltarget)+ (rdis - Rtarget))/2;
            drive.followTrajectorySequence(lineupMove);
            ldis = sensorDistanceL.getDistance(DistanceUnit.INCH);
            rdis = sensorDistanceR.getDistance(DistanceUnit.INCH);
        }
    }
    public void difinePaths(int z){
        //z1 = red right
        //z2 = blue left
        //z3 = red left
        //z4 = blue right
        double z1_1_p1_1x = 44;
        double z1_1_p1_1y = 9;
        double z1_1_p1_2x = 38;
        double z1_1_p1_2y = 7.5;
        double z1_1_p1_2a1 = 180;
        double z1_1_p1_2a2 = 180;
        double z1_1_p1_3x = 40;
        double z1_1_p1_3y = 11;
        double z1_1_p1_4x = 41;
        double z1_1_p1_4y = 14;
        double z1_1_p1_4a = 90;
        double z1_1_p1_5x = 36;
        double z1_1_p1_5y = 36;
        double z1_1_p1_5a1 = -135;
        double z1_1_p1_5a2 = 90;

        double z1_startpose_x = 61;
        double z1_startpose_y = 12;
        double z1_startpose_a= 135;

        if (z==2 || z==4){
            z1_1_p1_1x = flipX(z1_1_p1_1x);
            z1_1_p1_2x = flipX(z1_1_p1_2x);
            z1_1_p1_2a1 = flipBotYAngle(z1_1_p1_2a1);
            z1_1_p1_2a2 = flipFealdYAngle(z1_1_p1_2a2);
            z1_1_p1_3x = flipX(z1_1_p1_3x);
            z1_1_p1_4x = flipX(z1_1_p1_4x);
            z1_1_p1_5x = flipX(z1_1_p1_5x);
            z1_1_p1_5a1 = flipBotYAngle(z1_1_p1_5a1);
            z1_1_p1_5a2 = flipFealdYAngle(z1_1_p1_5a2);

        }
        if (z == 3 || z == 4) {
            z1_1_p1_1y = flipY(z1_1_p1_1y);
            z1_1_p1_2y = flipY(z1_1_p1_2y);
            z1_1_p1_2a1 = flipBotXAngle(z1_1_p1_2a1);
            z1_1_p1_2a2 = flipFealdXAngle(z1_1_p1_2a2);
            z1_1_p1_3y = flipY(z1_1_p1_3y);
            z1_1_p1_4y = flipY(z1_1_p1_4y);
            z1_1_p1_5y = flipY(z1_1_p1_5y);
            z1_1_p1_5a1 = flipBotYAngle(z1_1_p1_5a1);
            z1_1_p1_5a2 = flipFealdYAngle(z1_1_p1_5a2);
        }

        double z1_0_p2_1x = 36;
        double z1_0_p2_1y = 48;
        double z1_0_p2_2x = 12;
        double z1_0_p2_2y = 18;
        double z1_0_p2_2a = -90;
        double z1_0_p2_3x = 12;
        double z1_0_p2_3y = -60;
        double z1_0_p2_4x = 12;
        double z1_0_p2_4y = 9;
        double z1_0_p2_5x = 36;
        double z1_0_p2_5y = 36;
        double z1_0_p2_5a = 90;
        double z1_0_p2_6x = 36;
        double z1_0_p2_6y = 48;
        if (z == 2){
            z1_0_p2_1x = flipX(z1_0_p2_1x);
            z1_0_p2_2x = flipX(z1_0_p2_2x);
            z1_0_p2_2a = flipFealdYAngle(z1_0_p2_2a);
            z1_0_p2_3x = flipX(z1_0_p2_3x);
            z1_0_p2_4x = flipX(z1_0_p2_4x);
            z1_0_p2_5x = flipX(z1_0_p2_5x);
            z1_0_p2_5a = flipFealdYAngle(z1_0_p2_5a);
            z1_0_p2_6x = flipX(z1_0_p2_6x);
        }


        Pose2d z1_startpose = new Pose2d(z1_startpose_x,z1_startpose_y,Math.toRadians(z1_startpose_a));

        drive.setPoseEstimate(z1_startpose);

        z1_1_p1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(44, 9))
                .splineToSplineHeading(new Pose2d(38, 7, Math.toRadians(205)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                        }
                )
                .waitSeconds(0)
                .strafeTo(new Vector2d(40, 11))
                .splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30, 48, Math.toRadians(-135)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                .strafeTo(new Vector2d(12, -56))
                .waitSeconds(2)
                //.strafeTo(new Vector2d(12, 9))
                //.splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(90))
                //.strafeTo(new Vector2d(36, 48))
                .strafeTo(new Vector2d(12, 22))
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                .strafeTo(new Vector2d(12, -56))
                .waitSeconds(2)
                .strafeTo(new Vector2d(12, 22))
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                //.strafeTo(new Vector2d(36, 48))
                .build();
        /*z1_1_p1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(z1_1_p1_1x, z1_1_p1_1y))
                .splineToSplineHeading(new Pose2d(z1_1_p1_2x, z1_1_p1_2y, Math.toRadians(z1_1_p1_2a1)), Math.toRadians(z1_1_p1_2a2))
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                .waitSeconds(0)
                .strafeTo(new Vector2d(z1_1_p1_3x, z1_1_p1_3y))
                .splineToConstantHeading(new Vector2d(z1_1_p1_4x, z1_1_p1_4y), Math.toRadians(z1_1_p1_4a))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    inMotor.setPower(-.5);
                    // Run your action in here!
                })
                .splineToSplineHeading(new Pose2d(z1_1_p1_5x, z1_1_p1_5y, Math.toRadians(z1_1_p1_5a1)), Math.toRadians(z1_1_p1_5a2))
                // wright before the first time to the backdrop
                .strafeTo(new Vector2d(z1_0_p2_1x, z1_0_p2_1y))
                .addDisplacementMarker(() -> {
                    //dump
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    //put dump down
                })
                .splineToConstantHeading(new Vector2d(z1_0_p2_2x, z1_0_p2_2y), Math.toRadians(z1_0_p2_2a))
                .strafeTo(new Vector2d(z1_0_p2_3x, z1_0_p2_3y))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                .addDisplacementMarker(() -> {
                    //pick of stacks
                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    inMotor.setPower(-.5);
                })

                .strafeTo(new Vector2d(z1_0_p2_4x, z1_0_p2_4y))
                .splineToConstantHeading(new Vector2d(z1_0_p2_5x, z1_0_p2_5y), Math.toRadians(z1_0_p2_5a))
                .strafeTo(new Vector2d(z1_0_p2_6x, z1_0_p2_6y))
                //repeat
                /*.turn(0)
                .strafeTo(new Vector2d(z1_0_p2_1x, z1_0_p2_1y))
                .addDisplacementMarker(() -> {
                    //dump
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    //put dump down
                })
                .splineToConstantHeading(new Vector2d(z1_0_p2_2x, z1_0_p2_2y), Math.toRadians(z1_0_p2_2a))
                .strafeTo(new Vector2d(z1_0_p2_3x, z1_0_p2_3y))
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                .addDisplacementMarker(() -> {
                    //pick of stacks
                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    inMotor.setPower(-.5);
                })*/
                //.build();
    }
    public static double flipBotYAngle(double angle){

        return (angle - 45)*-1 + 45;
    }
    public static double flipFealdYAngle(double angle){

        return (angle - 90)*-1 + 90;
    }
    public static double flipBotXAngle(double angle){

        return (angle + 45)*-1 -45;
    }
    public static double flipFealdXAngle(double angle){

        return angle *-1;
    }
    public static double flipX(double X){

        return -X;
    }
    public static double flipY(double Y){

        return Y * -1 -24;
    }
    public static Vector<Double> getCammraPosWithAprilTag(AprilTagDetection detection){
        double cx = detection.ftcPose.x;
        double cy = detection.ftcPose.y;
        // cx & cy are in inches

        double yaw = detection.ftcPose.yaw;
        // yaw is in digress

        double range = detection.ftcPose.range;
        // range is in inches

        double thetaA = 180;
        if (detection.id<=6) thetaA = 0;
        // thetaA is in digress

        // the reason thees functions don't look right is because fieldPosition is rotated 90 digress from how we want it
        double ax = -detection.metadata.fieldPosition.get(1);
        double ay = detection.metadata.fieldPosition.get(0);
        // ax & ay are in inches

        double thetaCF = thetaA - yaw;
        // thetaCF is in digress

        if (cx == 0){
            cx = 0.0000001;
        }

        double alpha = Math.atan(cy/cx);
        // alpha is in radians

        if (alpha < 0){
            alpha += Math.PI;
        }

        double b = Math.toRadians(thetaCF) + alpha;
        // b is in radians

        double cxf = range * Math.cos(b);
        double cyf = range * Math.sin(b);
        // cxf & cyf are in inches


        //35.5, -70.5
        double by = ay - cyf;
        double bx = ax - cxf;
        // by & bx are in inches

        Vector<Double> output = new Vector<>();
        output.add(thetaCF);
        output.add(bx);
        output.add(by);
        output.add(ax);
        output.add(ay);
        output.add(thetaA);
        return output;
    }
    public static Vector<Double> getRobotPosFromCameraStream(double thetaCF, double by, double bx){
        //                                                          degrees
        double rx = -4;
        double ry = 2;
        double thetaCR = 45;
        // degrees
        double thetaRF = thetaCF - thetaCR;
        // degrees
        double range = Math.sqrt(Math.pow(ry, 2)+Math.pow(rx, 2));
        if (rx == 0) rx = 0.0000001;
        double alpha = Math.atan(ry/rx);
        // radians
        if (rx < 0){
            if (ry > 0){
                alpha += Math.PI;
            }else {
                alpha -= Math.PI;
            }
        }
        double b = Math.toRadians(thetaRF) + alpha;
        // radians
        double ryf = range * Math.sin(b);
        double rxf = range * Math.cos(b);
        double cy = by - ryf;
        double cx = bx - rxf;
        //thetaRF  += 90; // this is to change ware the zero is from up to right  ->

        Vector<Double> output = new Vector<>();
        output.add(thetaRF);
        output.add(cx);
        output.add(cy);
        return output;
    }

}
//It shows the location of the Tag.
