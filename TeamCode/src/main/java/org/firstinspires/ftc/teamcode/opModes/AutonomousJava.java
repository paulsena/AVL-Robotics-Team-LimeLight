/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.components.RobotConstants.MascotSpikePosition.LEFT;
import static org.firstinspires.ftc.teamcode.components.RobotConstants.MascotSpikePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.components.RobotConstants.MascotSpikePosition.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.RobotConstants;
import org.firstinspires.ftc.teamcode.components.Vision;

/*
 * Quick Autonomous code for team Limelight
 */

@Autonomous(name="Autonomous New")
@Config
public class AutonomousJava extends LinearOpMode {

    // Variables Constants To Set!!!
    // **************************************
    public static double GRIPPER_OPEN_POSITION = 0.4;
    public static double GRIPPER_CLOSED_POSITION = 0.1;
    public static double GRIPPER_WRIST_GOAL_POSITION = .35;
    public static double GRIPPER_WRIST_FLOOR_POSITION = .6;
    public static double GRIPPER_WRIST_FLAT_POSITION = .65;
    public static int FLOOR_ELBOW_POSITION_OFFSET = -920;
    // **************************************


    // Declare each of the 4 motors, 2 arm joints, and gripper
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor armElbow, armShoulder;
    private Servo gripRotate, grip;

    int startingElbowPosition;
    int startingShoulderPosition;

    RobotConstants.MascotSpikePosition mascotPos = MIDDLE;

    StrafeToGoal strafeToGoalDirection = StrafeToGoal.NONE;

    private Vision vision;

    private RobotConstants.TEAM_COLOR teamColor = RobotConstants.TEAM_COLOR.RED;

    // Config Movements
    // Center
    public static int moveBackToSpikeCenterTime = 1800;
    public static int moveForwardResetCenterTime = 1150;


    // Left
    public static int moveBackToSpikeLeftTime = 1200;
    public static int moveForwardSpikeLeftTime = 400;
    public static int rotateSpikeLeftTime = 1100;
    public static int moveForwardResetLeftTime = 500;


    // Right
    public static int moveBackToSpikeRightTime = 1100;
    public static int moveForwardSpikeRightTime = 250;

    public static int rotateSpikeRightTime = 1150;
    public static int moveForwardResetRightTime = 325;



    public static int strafeShortTime = 3500;
    public static int strafeLongTime = 6500;
    @Override
    public void runOpMode() {

        initializeAndWait();

        // Arm down for floor drop pos
        moveArmWait(null, startingElbowPosition + FLOOR_ELBOW_POSITION_OFFSET);

        // Gripper Up for floor drop position
        gripRotate.setPosition(GRIPPER_WRIST_FLOOR_POSITION);

        if (mascotPos == RIGHT) {
            // Drive backwards to line
            moveRobotWait(-.2, 0, 0, moveBackToSpikeRightTime);

            // Pivot Right
            moveRobotWait(0, 0, .2, rotateSpikeRightTime);

            // Open Gripper
            sleep(500);
            grip.setPosition(GRIPPER_OPEN_POSITION);
            sleep(1000);

            // Pivot Reset Left
            moveRobotWait(.2, 0 , 0, moveForwardSpikeLeftTime);
            moveRobotWait(0, 0, -.2, rotateSpikeRightTime);

            // Arm reset
            moveArmWait(null, startingElbowPosition);

            // Move forward to starting position
            moveRobotWait(.2, 0 , 0, moveForwardResetRightTime);
            sleep(1000);
        } else if (mascotPos == MIDDLE) {

            // Drive backwards to line
            moveRobotWait(-.2, 0, 0, moveBackToSpikeCenterTime);

            // Open Gripper
            sleep(500);
            grip.setPosition(GRIPPER_OPEN_POSITION);
            sleep(1000);

            // Arm reset
            moveArmWait(null, startingElbowPosition);

            // Move forward to starting position
            moveRobotWait(.2, 0 , 0, moveForwardResetCenterTime);
            sleep(1000);

        } else if (mascotPos == LEFT) {
            // Drive backwards to line
            moveRobotWait(-.2, 0, 0, moveBackToSpikeLeftTime);

            // Pivot Left
            moveRobotWait(0, 0, -.2, rotateSpikeLeftTime);

            // Open Gripper
            sleep(500);
            grip.setPosition(GRIPPER_OPEN_POSITION);
            sleep(1000);

            // Pivot Reset Right
            moveRobotWait(.2, 0 , 0, moveForwardSpikeRightTime);
            moveRobotWait(0, 0, .2, rotateSpikeLeftTime);

            // Arm reset
            moveArmWait(null, startingElbowPosition);

            // Move forward to starting position
            moveRobotWait(.2, 0 , 0, moveForwardResetLeftTime);
            sleep(1000);
        }


        //Strafe
        if (teamColor == RobotConstants.TEAM_COLOR.BLUE && strafeToGoalDirection == StrafeToGoal.LONG) {
            moveRobotWait(0, .4 , 0, strafeLongTime);
        } else if (teamColor == RobotConstants.TEAM_COLOR.BLUE && strafeToGoalDirection == StrafeToGoal.SHORT) {
            moveRobotWait(0, .4 , 0, strafeShortTime);
        } else if (teamColor == RobotConstants.TEAM_COLOR.RED && strafeToGoalDirection == StrafeToGoal.LONG) {
            moveRobotWait(0, -.4 , 0, strafeLongTime);
        } else if (teamColor == RobotConstants.TEAM_COLOR.RED && strafeToGoalDirection == StrafeToGoal.SHORT) {
            moveRobotWait(0, -.4 , 0, strafeShortTime);
        } else if (strafeToGoalDirection == StrafeToGoal.NONE) {
            // No Op
        }

        telemetry.addLine("Done");
        telemetry.update();
    }

    private void moveRobotWait(double forward, double strafe, double rotate, int waitMilli) {
        // Move for given wait time
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() <= waitMilli)) {
            moveRobot(forward, strafe , rotate);
        }
        // Stop moving
        moveRobot(0,0,0);
    }

    private void moveArmWait(Integer shoulder, Integer elbow) {
        if (shoulder != null) {
            armShoulder.setTargetPosition(shoulder);
            armShoulder.setPower(.2);
        }
        if (elbow != null) {
            armElbow.setTargetPosition(elbow);
            armElbow.setPower(.2);
        }

        while (opModeIsActive() && (armElbow.isBusy() || armShoulder.isBusy())) {
            telemetry.addLine("Moving Arm");
            telemetry.update();
            sleep(100);
        }
    }

    private void moveRobot(double forward, double strafe, double rotate) {
        double max;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower   = forward - strafe + rotate;
        double rightBackPower  = forward + strafe - rotate;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initializeAndWait() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        armElbow = hardwareMap.get(DcMotor.class, "armElbow");
        armShoulder = hardwareMap.get(DcMotor.class, "armShoulder");
        gripRotate = hardwareMap.get(Servo.class, "gripRotate");
        grip = hardwareMap.get(Servo.class, "grip");

        // Reverse motors on right side b/c of placement on robot
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sets up arm starting positions based on current rest state
        startingElbowPosition = armElbow.getCurrentPosition();
        startingShoulderPosition = armShoulder.getCurrentPosition();

        armElbow.setTargetPosition(startingElbowPosition);
        armShoulder.setTargetPosition(startingShoulderPosition);
        armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Sets gripper to closed and flat on init
        grip.setPosition(GRIPPER_CLOSED_POSITION);
        gripRotate.setPosition(GRIPPER_WRIST_FLAT_POSITION);

        // Reset run time
        runtime.reset();

        // Init Vision
        vision = new Vision(hardwareMap);

        // Wait here until play button pushed
        // Get settings for autonomous from gamepad
        while(!isStarted()) {
            mascotPos = vision.getMascotSpikeEnum(null);

            if (gamepad1.dpad_left) {
                mascotPos = LEFT;
            }
            else if (gamepad1.dpad_right) {
                mascotPos = RIGHT;
            }
            else if (gamepad1.dpad_up) {
                mascotPos = MIDDLE;
            }

            if (gamepad1.cross) {
                teamColor = RobotConstants.TEAM_COLOR.BLUE;
                vision.setTeamColor(teamColor);

                if (strafeToGoalDirection == StrafeToGoal.LONG) {
                    strafeToGoalDirection = StrafeToGoal.SHORT;
                } else {
                    strafeToGoalDirection = StrafeToGoal.LONG;
                }
            } else if (gamepad1.circle) {
                teamColor = RobotConstants.TEAM_COLOR.RED;
                vision.setTeamColor(teamColor);

                if (strafeToGoalDirection == StrafeToGoal.LONG) {
                    strafeToGoalDirection = StrafeToGoal.SHORT;
                } else {
                    strafeToGoalDirection = StrafeToGoal.LONG;
                }
            }
            if (gamepad1.triangle) {
                strafeToGoalDirection = StrafeToGoal.NONE;
            }

            // Instructions
            telemetry.addData("Mascot Position: ", mascotPos);
            telemetry.addLine();
            telemetry.addLine("Team Color  (Set with X or O): " + teamColor);
            telemetry.addData("Strafe (Set with X or O): ", strafeToGoalDirection);
            telemetry.addLine("Set Mascot with Dpad if vision is not working");
            telemetry.update();
        }
    }

    enum StrafeToGoal {
        LONG,
        SHORT,
        NONE
    }
}
