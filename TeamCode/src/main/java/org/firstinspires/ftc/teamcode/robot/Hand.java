package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: might need to reconfig, turny wrist goTo, fingers open/close

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

        //TODO: this stuff
//        leftClaw.setDirection();
//        rightClaw.setDirection();
//        leftClaw.scaleRange();
//        rightClaw.scaleRange();
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
     * @param open true if we want to open the claws, false otherwise
     */
    public void setClawsPos(boolean open) {
        leftClaw.setPosition(open ? 1.0 : 0.0);
        rightClaw.setPosition(open ? 1.0 : 0.0);
    }

    /**
     * Return the turny wrist Position
     *
     * @return the servo position [0, 1]
     */
    public double getTurnyWristPos() {
        return turnyWrist.getPosition();
    }
}
