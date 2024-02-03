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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(1), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61,12,Math.toRadians(135)))
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
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                //.strafeTo(new Vector2d(12, 9))
                                //.splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(90))
                                //.strafeTo(new Vector2d(36, 48))
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                                //.strafeTo(new Vector2d(36, 48))
                                .build()
                );
        RoadRunnerBotEntity myBotb = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(1), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61,12,Math.toRadians(135)))
                                .strafeTo(new Vector2d(33.5, 18))
                                .addDisplacementMarker(() -> {
                                            // This marker runs after the first splineTo()

                                            // Run your action in here!
                                        }
                                )
                                .waitSeconds(0)
                                .strafeTo(new Vector2d(36, 20.5))
                                .splineToSplineHeading(new Pose2d(36, 36, Math.toRadians(-135)), Math.toRadians(135))
                                .strafeTo(new Vector2d(36, 48))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                //.strafeTo(new Vector2d(12, 9))
                                //.splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(90))
                                //.strafeTo(new Vector2d(36, 48))
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(45))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(45))
                                //.strafeTo(new Vector2d(36, 48))
                                .build()
                );
        RoadRunnerBotEntity myBotc = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(1), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61,12,Math.toRadians(135)))
                                .strafeTo(new Vector2d(58, 15))
                                .splineToConstantHeading(new Vector2d(42, 23), Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                            // This marker runs after the first splineTo()

                                            // Run your action in here!
                                        }
                                )
                                .waitSeconds(0)
                                .strafeTo(new Vector2d(44, 26))
                                .splineToSplineHeading(new Pose2d(42, 48, Math.toRadians(-135)), Math.toRadians(90))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                //.strafeTo(new Vector2d(12, 9))
                                //.splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(90))
                                //.strafeTo(new Vector2d(36, 48))
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, -60))
                                .waitSeconds(2)
                                .strafeTo(new Vector2d(12, 22))
                                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(45))
                                //.strafeTo(new Vector2d(36, 48))
                                .build()
                );
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(flipX(61),12,Math.toRadians(flipBotYAngle(135))))
                                .strafeTo(new Vector2d(flipX(44), 9))
                                .splineToSplineHeading(new Pose2d(flipX(38), 7, Math.toRadians(flipBotYAngle(205))), Math.toRadians(flipFealdYAngle(180)))

                                .build()
                );
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61,flipY(12),Math.toRadians(flipBotXAngle(135))))
                                .strafeTo(new Vector2d(44, flipY(9)))
                                .splineToSplineHeading(new Pose2d(38, flipY(7), Math.toRadians(flipBotXAngle(205))), Math.toRadians(flipFealdXAngle(180)))

                                .build()
                );
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
        double z1_0_p2_7x = 12;
        double z1_0_p2_7y = 18;
        double z1_0_p2_8x = 36;
        double z1_0_p2_8y = 48;
        double z1_0_p2_8a = 45;
        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(60), 12.95)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,36,Math.toRadians(-135)))
                                .strafeTo(new Vector2d(z1_0_p2_1x, z1_0_p2_1y))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(z1_0_p2_2x, z1_0_p2_2y), Math.toRadians(z1_0_p2_2a))
                                .strafeTo(new Vector2d(z1_0_p2_3x, z1_0_p2_3y))
                                .addDisplacementMarker(() -> {
                                    // This marker runs after the first splineTo()
                                    // Run your action in here
                                })
                                .waitSeconds(2)

                                .strafeTo(new Vector2d(z1_0_p2_4x, z1_0_p2_4y))
                                .splineToConstantHeading(new Vector2d(z1_0_p2_5x, z1_0_p2_5y), Math.toRadians(z1_0_p2_5a))
                                .strafeTo(new Vector2d(z1_0_p2_6x, z1_0_p2_6y))
                                .build()
                );
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\jdkra\\StudioProjects\\FtcRobotController22-23\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\meepmeepCenterStage.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
        //meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotc)
                .addEntity(myBot2)
                .addEntity(myBot3)
                //.addEntity(myBot4)
                .start();
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
}