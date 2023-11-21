package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="test", group = "tests")
@Disabled
public class roadrunerstraghtest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startpose = new Pose2d(61,9,Math.toRadians(135));
        drive.setPoseEstimate(startpose);

        //Trajectory trajectory = drive.trajectoryBuilder(startpose)
        //        .forward(12)
                //.lineTo(new Vector2d(DISTANCE,DISTANCE))
        //        .build();

        //drive.setPoseEstimate(new Pose2d(61, 9, Math.toRadians(135)));
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startpose)
                .strafeTo(new Vector2d(40, 9))
                .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()
                            drive.setPoseEstimate(new Pose2d(40,15,Math.toRadians(135)));
                            // Run your action in here!
                        }
                )
                .strafeTo(new Vector2d(19, 9))
                .build();

        //Trajectory trajectory = drive.trajectoryBuilder(startpose)
        //        .strafeTo(new Vector2d(40,9))
        //        .build();

        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectory(trajectory);
        drive.followTrajectorySequence(ts);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        //telemetry.addData("heading error", poseEstimate.he)
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

