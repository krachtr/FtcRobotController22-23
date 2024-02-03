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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.Vector;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag", group = "Concept")
//@Disabled
public class ConceptAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
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
        //myAprilTagLibraryBuilder.addTag(10, "Our Awesome Team Tag", 5, DistanceUnit.INCH);

        // Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
        //aprilTag = myAprilTagProcessorBuilder.build();

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
            .setLensIntrinsics(879.145, 879.145, 297.525, 260.564)

            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        /*  see https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_init/visionportal-init.html
            for other options of creating the vision portal
            cpu management: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_cpu_and_bandwidth/visionportal-cpu-and-bandwidth.html

            should: builder.enableLiveView(false)?
                    myVisionPortal.stopStreaming();
                    myVisionPortal.close();
         */



        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine("");

                Vector<Double> output = getCammraPosWithAprilTag(detection);
                telemetry.addData("tag x", output.get(3));
                telemetry.addData("tag y", output.get(4));
                telemetry.addData("tag a", output.get(5));
                telemetry.addLine("");
                telemetry.addData("cam x", output.get(1));
                telemetry.addData("cam Y", output.get(2));
                telemetry.addData("cam A", output.get(0));
                telemetry.addLine("");
                output = getRobotPosFromCameraStream(output.get(0), output.get(2), output.get(1));
                telemetry.addData("bot x", output.get(1));
                telemetry.addData("bot Y", output.get(2));
                telemetry.addData("bot A", output.get(0));
                telemetry.addLine("");


                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public static double calculateAngle(double xc, double yc, double xt, double yt, double range, double yaw) {
        // Calculate the angle in radians
        double angleRadians = Math.atan2(yt - yc, xt - xc) - Math.toRadians(yaw);

        // Convert the angle to degrees
        double angleDegrees = Math.toDegrees(angleRadians);

        // Ensure the angle is within [0, 360) range
        angleDegrees = (angleDegrees + 360) % 360;

        return angleDegrees;
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

}   // end class
