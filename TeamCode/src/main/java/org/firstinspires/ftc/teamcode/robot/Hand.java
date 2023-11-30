package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: wrist PID, turny wrist goTo, fingers open/close

/**
 * The Hand Subsystem
 */
public class Hand implements Constants {
    private DcMotorEx wristMotor;
    private Servo turnyWrist;
    private Servo leftClaw;
    private Servo rightClaw;

    /**
     * Initializes the Hand object
     *
     * @param hwMap the hardwareMap
     */
    public void init(HardwareMap hwMap) {
        wristMotor = hwMap.get(DcMotorEx.class, "wristMotor");

        turnyWrist = hwMap.get(Servo.class, "turnyWrist");
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
    }

    /**
     * Moves the wrist manually
     *
     * @param power the turning power [-max_speed, max_speed]
     */
    public void wristManualControl(double power) {
        power = Range.clip(power, -WRIST_GOVERNOR, WRIST_GOVERNOR);
        wristMotor.setPower(power);
    }
}
