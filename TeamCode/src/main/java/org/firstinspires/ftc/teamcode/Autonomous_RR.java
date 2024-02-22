package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Autonomous_RR extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Init
        Pose2d beginPose = new Pose2d(-84.7288, 112.93162, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, 19.25))
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-55.38, 19.25), Math.toRadians(90))
                        .build());
    }
}
