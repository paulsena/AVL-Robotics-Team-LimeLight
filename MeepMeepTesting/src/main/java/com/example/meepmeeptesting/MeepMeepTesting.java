package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d starting = new Pose2d(0, 0, 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 26.2044)

                .followTrajectorySequence(drive -> {

                    TrajectorySequence driveToGoal = drive.trajectorySequenceBuilder(new Pose2d(-35.97, 57.89, Math.toRadians(270.00)))
                            .splineTo(new Vector2d(-52.17, 47.14), Math.toRadians(219.85))
                            .splineTo(new Vector2d(-57.47, 19.48), Math.toRadians(264.83))
                            .splineTo(new Vector2d(-42.39, 12.22), Math.toRadians(0))
                            .lineToSplineHeading(new Pose2d(25.91, 12.50, Math.toRadians(0)))
                            .splineTo(new Vector2d(42.25, 34.99), Math.toRadians(0.00))
                            .build();

                        return driveToGoal;
                        }
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}