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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

@Autonomous(name="Auto Meat 1", group = "A")

public class AutoMeat1 extends LinearOpMode
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

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 10;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode() throws InterruptedException {
        int program = 1;
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
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  driveForword    = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        int zone = 1;

        // Initialize the Apriltag Detection process
        //initAprilTag();



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startpose = new Pose2d(61,9,Math.toRadians(135));
        Pose2d startpose2 = new Pose2d(33,15,Math.toRadians(135));
        drive.setPoseEstimate(startpose);

        TrajectorySequence redRight1p1 = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(33, 9))
                // .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                //        }
                //)

                //.strafeTo(new Vector2d(36, 9))
                //.splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight1p2 = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(36, 9))
                .splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight2p1 = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(33, 15))
                //.addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                //        }
                //)
                //.strafeTo(new Vector2d(43, 12))
                //.splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight2p2 = drive.trajectorySequenceBuilder(startpose2)
                .strafeTo(new Vector2d(43, 12))
                .splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight3p1 = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(58, 9))
                .splineToConstantHeading(new Vector2d(48, 24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                        }
                )
                //.strafeTo(new Vector2d(51, 24))
                //.splineToConstantHeading(new Vector2d(50, 30), Math.toRadians(175))
                //.splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence redRight3p2 = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(51, 24))
                .splineToConstantHeading(new Vector2d(50, 30), Math.toRadians(175))
                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                .build();

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

        // Wait for driver to press start
        PropDetector propDetector = new PropDetector(hardwareMap);
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
        waitForStart();

        if (program == 1 || program == 2) {
            //Blue
            zone = propDetector.getTargetZone(1);
        } else {
            //Red
            zone = propDetector.getTargetZone(0);
        }


        if (program == 1) {
            telemetry.addData("", "");
        } else if (program == 2) {
            telemetry.addData("", "");
        } else if (program == 3) {
            telemetry.addData("", "");
        } else {
            if (zone == 1){
                //drive.followTrajectory(redRight1p1);
                //drive.followTrajectory(redRight1p2);
            } else if (zone == 2) {
                //drive.followTrajectory(redRight2p1);
                drive.followTrajectorySequence(redRight2p1);
                drive.followTrajectorySequence(redRight2p2);
            } else {
                //drive.followTrajectory(redRight3p1);
                //drive.followTrajectory(redRight3p2);
            }
        }
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
}
