package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo gripperWrist;
    private Servo gripperFingerLeft;
    private Servo gripperFingerRight;

    private static final double WRIST_START_POSITION = 0.8;
    private static final double WRIST_GOAL_POSITION = 0.55;
    private static final double WRIST_PICKUP_POSITION = 0.88;
    private static final double FINGER_OPEN_POSITION = 0.50;
    private static final double FINGER_CLOSE_POSITION = 0.00;


    public Gripper(HardwareMap hardwareMap) {
        gripperWrist = hardwareMap.get(Servo.class, "gripRotate");
        gripperFingerLeft = hardwareMap.get(Servo.class, "grip");
        gripperFingerRight = hardwareMap.get(Servo.class, "grip2");
    }

    public Action open (Finger finger) {
        return telemetryPacket -> {
            if (finger == Finger.LEFT_FINGER_YELLOW) {
                gripperFingerLeft.setPosition(FINGER_OPEN_POSITION);
            } else if (finger == Finger.RIGHT_FINGER_PURPLE) {
                gripperFingerRight.setPosition(FINGER_OPEN_POSITION);
            }
            return false;
        };
    }

    public Action close () {
        return telemetryPacket -> {
            gripperFingerLeft.setPosition(FINGER_CLOSE_POSITION);
            gripperFingerRight.setPosition(FINGER_CLOSE_POSITION);
            return true;
        };

    }

    public Action wristGoalPosition() {
        return telemetryPacket -> {
            gripperWrist.setPosition(WRIST_GOAL_POSITION);
            return true;
        };
    }

    public Action wristFloorPosition() {
        return telemetryPacket -> {
            gripperWrist.setPosition(WRIST_PICKUP_POSITION);
            return true;
        };
    }

    public enum Finger {
        LEFT_FINGER_YELLOW,
        RIGHT_FINGER_PURPLE
    }

    public Action init() {
        return (telemetryPacket) -> {
            gripperFingerLeft.setPosition(FINGER_CLOSE_POSITION);
            gripperFingerLeft.setPosition(FINGER_CLOSE_POSITION);
            gripperWrist.setPosition(WRIST_START_POSITION);
            return true;
        };
    }


}