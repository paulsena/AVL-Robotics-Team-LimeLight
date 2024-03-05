package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotConstants;
import org.firstinspires.ftc.teamcode.components.Vision;

@Autonomous(name="Vision Test")
public class VisionTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Vision vision = new Vision(hardwareMap);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() || !isStarted()) {

            if(gamepad1.dpad_left) {
                vision.setTeamColor(RobotConstants.TEAM_COLOR.RED);
                //telemetry.addData("Team Color", "Red");
            } else if (gamepad1.dpad_right) {
                vision.setTeamColor(RobotConstants.TEAM_COLOR.BLUE);
                //telemetry.addData("Team Color", "Blue");
            }

            vision.getMascotSpikePosition(telemetry);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}