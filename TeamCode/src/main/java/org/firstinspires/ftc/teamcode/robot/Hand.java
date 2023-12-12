package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Hand Subsystem
 */
public class Hand implements Constants {
    private final Servo turnyWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;

    /**
     * Initializes the Hand object
     *
     * @param hwMap the hardwareMap
     */
    public Hand(HardwareMap hwMap) {
        turnyWrist = hwMap.get(Servo.class, "turnyWrist");
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");

        turnyWrist.scaleRange(0.0, MAX_TURNY_WRIST);
        leftClaw.scaleRange(0.0, LEFT_CLAW_OPEN);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.scaleRange(RIGHT_CLAW_OPEN, 1.0);

        turnyWrist.setPosition(0.5);
    }

    /**
     * Moves the wrist manually
     *
     * @param position the position to go to [min, max]
     */
    public void setTurnyWristPos(double position) {
        turnyWrist.setPosition(position);
    }

    /**
     * Moves the wrist manually
     *
     * @param open whether to open or close
     */
    public void setClawPos(boolean open) {
        leftClaw.setPosition(open ? 1.0 : 0.0);
        rightClaw.setPosition(open ? 1.0 : 0.0);
    }
}
