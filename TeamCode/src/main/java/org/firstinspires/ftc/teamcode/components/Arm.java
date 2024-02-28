package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final DcMotorEx shoulder;
    private final DcMotorEx elbow;

    public static int starting_shoulder_pos;
    public static int starting_elbow_pos;

    public static final double SHOULDER_SPEED = .6;
    public static final double ELBOW_SPEED = .3;
    public static final int GOAL_SHOULDER_OFFSET = -2635;
    public static final int GOAL_ELBOW_OFFSET = 250;
    public static final int FLOOR_ELBOW_OFFSET = -830;

    public Arm(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotorEx.class, "armShoulder");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow = hardwareMap.get(DcMotorEx.class, "armElbow");
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Action goalPosition() {
        return telemetryPacket -> {
            shoulder.setPower(.5);
            shoulder.setTargetPosition(starting_shoulder_pos + GOAL_SHOULDER_OFFSET);
            elbow.setTargetPosition(starting_elbow_pos + GOAL_ELBOW_OFFSET);
            return true;
        };
    }

    public Action floorPosition() {
        return telemetryPacket -> {
            elbow.setTargetPosition(starting_elbow_pos + FLOOR_ELBOW_OFFSET);
            return true;
        };
    }

    public Action resetPosition(int finger) {
        return telemetryPacket -> {
            shoulder.setTargetPosition(starting_shoulder_pos);
            elbow.setTargetPosition(starting_elbow_pos);
            return true;
        };
    }

    public Action init() {
        return telemetryPacket -> {
            starting_shoulder_pos = shoulder.getCurrentPosition();
            starting_elbow_pos = elbow.getCurrentPosition();
            shoulder.setPower(SHOULDER_SPEED);
            elbow.setPower(ELBOW_SPEED);
            return true;
        };
    }
}
