package org.firstinspires.ftc.teamcode.opModes;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Gripper;
import org.firstinspires.ftc.teamcode.components.RobotConstants;
import org.firstinspires.ftc.teamcode.components.Vision;

@Config
@Autonomous(name = "Autonomous RR", preselectTeleOp="Main Control")
public class AutonomousRROpMode extends LinearOpMode {


    @Override
    public void runOpMode() {

        // Init
        Pose2d beginPose = RobotConstants.StartingPositions.BLUE_FAR.getStartingPose();
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Arm arm = new Arm(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);
        drive.updatePoseEstimate();

        // Predefined Actions
        Action driveToGoalAction = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToX(12)
                .build();

        // Vision
        Vision vision = new Vision(hardwareMap);
        vision.getMascotSpikePosition(telemetry);

//            .stopAndAdd(arm.goalPosition())
//            .stopAndAdd(new SequentialAction(
//                    gripper.wristGoalPosition(),
//                    gripper.open(Gripper.Finger.RIGHT_FINGER_PURPLE)))


        // Place pixels then hit Init to close gripper
//        Actions.runBlocking(new SequentialAction(
//                arm.init(),
//                gripper.init(),
//                gripper.close()));

//
        while (!isStopRequested() && !opModeIsActive()) {
            RobotConstants.MascotSpikePosition mascotPosition = detectMascotPosition();
            telemetry.addData("Position during Init", mascotPosition);
            telemetry.addLine("Press Start at Buzzer");
            telemetry.update();
        }

        //Actions.runBlocking(driveToSpike);

        //Actions.runBlocking(driveToGoalAction);

    }

    // Use OpenCV to detect mascot starting position
    public RobotConstants.MascotSpikePosition detectMascotPosition() {
        //TODO Implement vision
        return RobotConstants.MascotSpikePosition.MIDDLE;
    }

}
