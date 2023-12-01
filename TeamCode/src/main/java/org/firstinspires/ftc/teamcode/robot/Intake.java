package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Intake Subsystem of the Robot
 */
public class Intake implements Constants {

    private final Servo leftServo;
    private final Servo rightServo;

    /** Initializes pan servos (linear actuators)
     *  MUST BE CALLED DURING TELEOP INIT OR ELSE
     */
    public Intake(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "leftDust");
        rightServo = hwMap.get(Servo.class, "rightDust");
    }

    /**
     * Sets pan actuators to the position
     *
     * @param isRaising whether to raise or lower the intake
     */
    public void panToPosition(boolean isRaising) {
        leftServo.setPosition(isRaising ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
        rightServo.setPosition(isRaising ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
    }

    /**
     * Get the servo positions for telemetry
     *
     * @return the servo position (specifically the left one)
     */
    public double getPosition() {
        return leftServo.getPosition();
    }
}