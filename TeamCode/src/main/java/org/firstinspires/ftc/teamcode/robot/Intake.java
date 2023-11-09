package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Intake Subsystem of the Robot
 */
public class Intake implements Constants {

    private Servo leftServo;
    private Servo rightServo;

    /** Initializes pan servos (linear actuators)
     *  MUST BE CALLED DURING TELEOP INIT OR ELSE
     */
    public void init(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "leftDust");
        rightServo = hwMap.get(Servo.class, "rightDust");
    }

    /**
     * Sets pan actuators to the position
     */
    public void panToPosition(boolean upPosition) {
        leftServo.setPosition(upPosition ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
        rightServo.setPosition(upPosition ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
    }

    /**
     * Get the servo positions for telemetry
     *
     * @return the servo position (left one)
     */
    public double getPosition() {
        return leftServo.getPosition();
    }

}