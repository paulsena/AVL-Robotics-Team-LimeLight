package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotConstants {

    public enum StartingPositions {

        BLUE_FAR(new Pose2d(-35.97, 57.89, Math.toRadians(270.00))),
        BLUE_NEAR(new Pose2d(-35.97, 57.89, Math.toRadians(270.00))),
        RED_FAR(new Pose2d(-35.97, 57.89, Math.toRadians(270.00))),
        RED_NEAR(new Pose2d(-35.97, 57.89, Math.toRadians(270.00)));
        private Pose2d startingPose;

        StartingPositions(Pose2d startingPose) {
            this.startingPose = startingPose;
        }

        public Pose2d getStartingPose() {
            return startingPose;
        }
    }

    public enum MascotSpikePosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum TEAM_COLOR {
        RED,
        BLUE
    }
}
