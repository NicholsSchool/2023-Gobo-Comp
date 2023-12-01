package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: reconfig bc it's wrong YAY, tune wrist goTo, turny wrist goTo, fingers open/close
//TODO: wrist forbar once pot is figured out

/**
 * The Hand Subsystem
 */
public class Hand implements Constants {
    private final DcMotorEx wristMotor;
    private final Servo turnyWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;

    /**
     * Initializes the Hand object
     *
     * @param hwMap the hardwareMap
     */
    public Hand(HardwareMap hwMap) {
        wristMotor = hwMap.get(DcMotorEx.class, "wrist");
        wristMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wristMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wristMotor.setPositionPIDFCoefficients(WRIST_P);

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
     * @param power the turning power [-max_speed, max_speed]
     */
    public void wristManualControl(double power) {
        power = Range.clip(power, -WRIST_GOVERNOR, WRIST_GOVERNOR);
        wristMotor.setPower(power);
    }


    /**
     * Turns the wrist to the specified position using PID
     *
     * @param position the encoder target position
     */
    public void setWristPos(int position) {
        wristMotor.setTargetPosition(position);
    }

    /**
     * Moves the wrist manually
     *
     * @param position the position to go to [servo min, max]
     */
    public void setTurnyWristPos(double position) {
        turnyWrist.setPosition(position);
    }

    /**
     * Moves the wrist manually
     *
     * @param open true if we want to open the claws, false otherwise
     */
    public void setClaws(boolean open) {
        leftClaw.setPosition(open ? 1.0 : 0.0);
        rightClaw.setPosition(open ? 1.0 : 0.0);
    }

    /**
     * Return the Wrist Position
     *
     * @return the motor's encoder position
     */
    public int getWristPos() {
        return wristMotor.getCurrentPosition();
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
