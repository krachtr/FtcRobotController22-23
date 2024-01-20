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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="Auto Meat 3", group = "A")
@Disabled
public class AutoMeat3 extends LinearOpMode
{

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backDrive = null;  //  Used to control the right back drive wheel

    private DcMotor inMotor = null;
    private DcMotor outMotor = null;
    private DcMotor graberMotor = null;
    //private DcMotor upMotor = null;

    public Servo shooter;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 10;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //for cam
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 7; // Tag ID 18 from the 36h11 family

    org.openftc.apriltag.AprilTagDetection tagOfInterest = null;






    @Override public void runOpMode() throws InterruptedException {
//This is initializing The intake servos.
        int grabUpPos = 26594;
        int grabHangPos = 15661;
        int grabCamPos = 3395;
        int grabDownPos = 0;

        int outLoadPos = 0;
        int outDropPos = -580;
        int outLineupPos = -250;

        int upRedyPos = 419;
        int upStartPos = 1600;
        inMotor = hardwareMap.get(DcMotor.class, "in");
        outMotor = hardwareMap.get(DcMotor.class, "out");
        graberMotor = hardwareMap.get(DcMotor.class, "grab");
        //upMotor = hardwareMap.get(DcMotor.class, "up");

        inMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        graberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.get(Servo.class, "shooter");

        inMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outMotor.setTargetPosition(outLoadPos);
        outMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //upMotor.setTargetPosition(upStartPos);
        //pMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        graberMotor.setTargetPosition(grabCamPos);
        graberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//This is initializing The arm motor.

        int program = 1;
        telemetry.addData("sulect a starting position whith the dPad: ", "blue left");
        telemetry.update();
        while (!gamepad1.a){
            if(gamepad1.dpad_up){
                program ++;
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
                while (gamepad1.dpad_up){
                    Thread.sleep(10);
                }
            } else if (gamepad1.dpad_down) {
                program --;
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
                while (gamepad1.dpad_down){
                    Thread.sleep(10);
                }
            }
            if (program < 1){
                program = 4;
            } else if (program > 4) {
                program = 1;
            }

            Thread.sleep(10);
        }
//This tells the robot where on the field it is starting.
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  driveForword    = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        int zone = 1;

        // Initialize the Apriltag Detection process
        //initAprilTag();



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startposerr = new Pose2d(61,12,Math.toRadians(135));
        Pose2d startpose1rr = new Pose2d(38, 7.5,Math.toRadians(180));
        Pose2d startpose2rr = new Pose2d(33.5, 18,Math.toRadians(135));
        Pose2d startpose3rr = new Pose2d(42, 23, Math.toRadians(125));

        Pose2d startposerr2 = new Pose2d(36, 36, Math.toRadians(-135));
        Pose2d startposebl2 = new Pose2d(-36, 36, Math.toRadians(-135));

        Pose2d startposebl = new Pose2d(-61,12,Math.toRadians(-45));
        Pose2d startpose1bl = new Pose2d(-42, 25, Math.toRadians(-45));
        Pose2d startpose2bl  = new Pose2d(-33.5, 18,Math.toRadians(-45));
        Pose2d startpose3bl = new Pose2d(-38, 7.5,Math.toRadians(-105));

        Pose2d startposebr = new Pose2d(61,12,Math.toRadians(135));
        Pose2d startpose1br = new Pose2d(38, 7,Math.toRadians(180));
        Pose2d startpose2br  = new Pose2d(32.5, 18,Math.toRadians(135));
        Pose2d startpose3br = new Pose2d(42, 23, Math.toRadians(125));

        Pose2d startposerl = new Pose2d(61,12,Math.toRadians(135));
        Pose2d startpose1rl = new Pose2d(38, 7,Math.toRadians(180));
        Pose2d startpose2rl  = new Pose2d(32.5, 18,Math.toRadians(135));
        Pose2d startpose3rl = new Pose2d(42, 23, Math.toRadians(125));
// This is all the possible places that the robot could start.
        if (program == 1) {
            // blue left, closest to back bord
            drive.setPoseEstimate(startposebl);
        } else if (program == 2) {
            // blue right, farthest to back bord
            drive.setPoseEstimate(startposerr);
        } else if (program == 3) {
            // red left, farthest to back bord
            drive.setPoseEstimate(startposebl);
        } else {
            // red right, closest to back bord
            drive.setPoseEstimate(startposerr);
        }

        TrajectorySequence redRight1p1 = drive.trajectorySequenceBuilder(startposerr)
                .strafeTo(new Vector2d(44, 9))
                .splineToSplineHeading(new Pose2d(38, 7.5, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })

                //.strafeTo(new Vector2d(36, 9))
                //.splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight1p2 = drive.trajectorySequenceBuilder(startpose1rr)
                .strafeTo(new Vector2d(40, 11))
                .splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-135)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight2p1 = drive.trajectorySequenceBuilder(startposerr)
                .strafeTo(new Vector2d(33.5, 18))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                //.strafeTo(new Vector2d(43, 12))
                //.splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight2p2 = drive.trajectorySequenceBuilder(startpose2rr)
                //.strafeTo(new Vector2d(43, 12))
                //.splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))

                .strafeTo(new Vector2d(36, 20.5))
                .splineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-135)), Math.toRadians(135))
                .build();
        TrajectorySequence redRight3p1 = drive.trajectorySequenceBuilder(startposerr)
                .strafeTo(new Vector2d(58, 15))
                .splineToConstantHeading(new Vector2d(42, 23), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                //.strafeTo(new Vector2d(51, 24))
                //.splineToConstantHeading(new Vector2d(50, 30), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight3p2 = drive.trajectorySequenceBuilder(startpose3rr)
                .strafeTo(new Vector2d(45, 25))
                //.splineToConstantHeading(new Vector2d(46, 28), Math.toRadians(175))
                .splineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-135)), Math.toRadians(180))
                .build();




        TrajectorySequence blueLeft1p1 = drive.trajectorySequenceBuilder(startposebl)
                .strafeTo(new Vector2d(-58, 15))
                .splineToConstantHeading(new Vector2d(-42, 25), Math.toRadians(90))
                .addDisplacementMarker(22, () -> {
                    // This marker runs 20 inches into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })

                //.strafeTo(new Vector2d(36, 9))
                //.splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence blueLeft1p2 = drive.trajectorySequenceBuilder(startpose1bl)
                .strafeTo(new Vector2d(-45, 25))
                //.splineToConstantHeading(new Vector2d(46, 28), Math.toRadians(175))
                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(-135)), Math.toRadians(0))
                .build();
        TrajectorySequence blueLeft2p1 = drive.trajectorySequenceBuilder(startposebl)
                .strafeTo(new Vector2d(-33.5, 18))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                //.strafeTo(new Vector2d(43, 12))
                //.splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence blueLeft2p2 = drive.trajectorySequenceBuilder(startpose2bl)
                //.strafeTo(new Vector2d(43, 12))
                //.splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))

                .strafeTo(new Vector2d(-35, 20.5))
                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(-135)), Math.toRadians(45))
                .build();
        TrajectorySequence blueLeft3p1 = drive.trajectorySequenceBuilder(startposebl)
                .strafeTo(new Vector2d(-44, 9))
                .splineToSplineHeading(new Pose2d(-38, 7.5, Math.toRadians(-105)), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    inMotor.setPower(.5);
                    // Run your action in here!
                })
                .build();
        TrajectorySequence blueLeft3p2 = drive.trajectorySequenceBuilder(startpose3bl)
                .strafeTo(new Vector2d(-40, 11))
                .splineToConstantHeading(new Vector2d(-41, 14), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(-135)), Math.toRadians(90))
                .build();


        TrajectorySequence redRight1p3 = drive.trajectorySequenceBuilder(startposerr2)
                .strafeTo(new Vector2d(28.5, 52.5))
                .build();

        TrajectorySequence blueLeft1p3 = drive.trajectorySequenceBuilder(startposebl2)
                .strafeTo(new Vector2d(-40, 52.5))
                .build();

        TrajectorySequence redRight2p3 = drive.trajectorySequenceBuilder(startposerr2)
                .strafeTo(new Vector2d(36, 52.5))
                .build();

        TrajectorySequence blueLeft2p3 = drive.trajectorySequenceBuilder(startposebl2)
                .strafeTo(new Vector2d(-34, 52.5))
                .build();

        TrajectorySequence redRight3p3 = drive.trajectorySequenceBuilder(startposerr2)
                .strafeTo(new Vector2d(42, 52.5))
                .build();

        TrajectorySequence blueLeft3p3 = drive.trajectorySequenceBuilder(startposebl2)
                .strafeTo(new Vector2d(-26.5, 52.5))
                .build();
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
            telemetry.addData(">", "blue left program sulected");
        } else if (program == 2) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "blue right program sulected");
        } else if (program == 3) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "red left program sulected");
        } else {
            //drive.setPoseEstimate(new Pose2d(61, 9, Math.toRadians(135)));
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            telemetry.addData(">", "red right program sulected");
        }
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
//This shows what program we have selected.
// for cam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//This is initializing the camera.
        while (!isStarted() && !isStopRequested())
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
        if(tagOfInterest != null)
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
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.pose.x <= 20)
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
        }




        waitForStart();

        if (program == 1 || program == 2) {
            //Blue
            PoseStorage.redSide = false;
            zone = aprilTagDetectionPipeline.getTargetZone(1);
        } else {
            //Red
            zone = aprilTagDetectionPipeline.getTargetZone(0);
            PoseStorage.redSide = true;
        }
        //shooter.setPosition(.75);
        ((DcMotorEx) graberMotor).setVelocity(10000);
        outMotor.setPower(.6);
        //upMotor.setPower(1);
        graberMotor.setTargetPosition(grabDownPos);
//This tells the robot to look for the red or the blue prop.

        if (program == 1) {
            //bl
            if (zone == 1){
                drive.followTrajectorySequence(blueLeft1p1);
                drive.followTrajectorySequence(blueLeft1p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(blueLeft1p3);
                outMotor.setTargetPosition(outDropPos);
            } else if (zone == 2) {
                //drive.followTrajectory(redRight2p1);
                drive.followTrajectorySequence(blueLeft2p1);
                drive.followTrajectorySequence(blueLeft2p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(blueLeft2p3);
                outMotor.setTargetPosition(outDropPos);

            } else {
                drive.followTrajectorySequence(blueLeft3p1);
                drive.followTrajectorySequence(blueLeft3p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(blueLeft3p3);
                outMotor.setTargetPosition(outDropPos);
            }
            sleep(1000);
            //upMotor.setTargetPosition(upRedyPos);
            outMotor.setTargetPosition(outLoadPos);
//This is for the movement of the robot on the blue side.
        } else if (program == 2) {
            //br
            if (zone == 1){
                drive.followTrajectorySequence(redRight1p1);
                drive.followTrajectorySequence(redRight1p2);
            } else if (zone == 2) {
                //drive.followTrajectory(redRight2p1);
                drive.followTrajectorySequence(redRight2p1);
                drive.followTrajectorySequence(redRight2p2);

            } else {
                drive.followTrajectorySequence(redRight3p1);
                drive.followTrajectorySequence(redRight3p2);
            }

        } else if (program == 3) {
            //rl
            if (zone == 1){
                drive.followTrajectorySequence(blueLeft1p1);
                drive.followTrajectorySequence(blueLeft1p2);
            } else if (zone == 2) {
                //drive.followTrajectory(redRight2p1);
                drive.followTrajectorySequence(blueLeft2p1);
                drive.followTrajectorySequence(blueLeft2p2);

            } else {
                drive.followTrajectorySequence(blueLeft3p1);
                drive.followTrajectorySequence(blueLeft3p2);
            }
        } else {
            //rr
            if (zone == 1){
                drive.followTrajectorySequence(redRight1p1);
                drive.followTrajectorySequence(redRight1p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(redRight1p3);
                outMotor.setTargetPosition(outDropPos);
            } else if (zone == 2) {
                //drive.followTrajectory(redRight2p1);
                drive.followTrajectorySequence(redRight2p1);
                drive.followTrajectorySequence(redRight2p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(redRight2p3);
                outMotor.setTargetPosition(outDropPos);
            } else {
                drive.followTrajectorySequence(redRight3p1);
                drive.followTrajectorySequence(redRight3p2);
                inMotor.setPower(-.5);
                drive.followTrajectorySequence(redRight3p3);
                outMotor.setTargetPosition(outDropPos);
            }
            sleep(1000);
            //.setTargetPosition(upRedyPos);
            outMotor.setTargetPosition(outLoadPos);
        }
//This is for the movement of the robot on the red side.
        PoseStorage.currentPose = drive.getPoseEstimate();
        sleep(4000);

        /*while (opModeIsActive())
        {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null)
                        && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))  ){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }


            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                driveForword  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveForword, strafe, turn);
            }

            // Apply desired axes motions to the drivetrain.
            moveRobot(driveForword, strafe, turn);
            sleep(10);
        }*/
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
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        //aprilTag = new AprilTagProcessor.Builder().build();
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;
        AprilTagProcessor myAprilTagProcessor;

        // Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add a tag, without pose information, to the AprilTagLibrary.Builder.
       // myAprilTagLibraryBuilder.addTag(9, "Our Awesome Team Tag", 2, DistanceUnit.INCH);

        // Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
        aprilTag = myAprilTagProcessorBuilder.build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

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
}
//It shows the location of the Tag.
