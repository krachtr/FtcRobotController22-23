package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 9, Math.toRadians(135)))
                                .strafeTo(new Vector2d(33, 9))
                                .addDisplacementMarker(() -> {
                                    // This marker runs after the first splineTo()

                                    // Run your action in here!
                                }
                                )
                                .waitSeconds(0)
                                .strafeTo(new Vector2d(36, 9))
                                .splineToConstantHeading(new Vector2d(41, 14), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 9, Math.toRadians(135)))
                                .lineToLinearHeading(new Pose2d(40, 9, Math.toRadians(180)))
                                .addDisplacementMarker(() -> {
                                            // This marker runs after the first splineTo()

                                            // Run your action in here!
                                        }
                                )
                                .waitSeconds(0)
                                .strafeTo(new Vector2d(43, 12))
                                .splineToConstantHeading(new Vector2d(43, 20), Math.toRadians(175))
                                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 9, Math.toRadians(135)))
                                .strafeTo(new Vector2d(58, 9))
                                .splineToConstantHeading(new Vector2d(48, 24), Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                            // This marker runs after the first splineTo()

                                            // Run your action in here!
                                        }
                                )
                                .waitSeconds(0)
                                .strafeTo(new Vector2d(51, 24))
                                .splineToConstantHeading(new Vector2d(50, 30), Math.toRadians(175))
                                .splineToSplineHeading(new Pose2d(36, 35, Math.toRadians(45)), Math.toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 9, Math.toRadians(135)))
                                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(135)))
                                .build()
                );
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\jdkra\\StudioProjects\\FtcRobotController22-23\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\meepmeepCenterStage.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
        //meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .start();
    }
}