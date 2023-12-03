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
     * does pan movement without getting in the way of the arm
     * @param isLowering is the driver lowering the pan
     * @param potAngle angle of the arm
     * @param deltaPot is the arm going up or down
     */
    public void panToPosition(boolean isLowering, double potAngle, double deltaPot) {
        if(isLowering) {
            leftServo.setPosition(INTAKE_DOWN_POSITION);
            rightServo.setPosition(INTAKE_DOWN_POSITION);
        }else if((potAngle > PAN_ARM_UP_LIMIT && deltaPot < 0) || (potAngle > PAN_ARM_DOWN_LIMIT)){
            leftServo.setPosition(INTAKE_UP_POSITION);
            rightServo.setPosition(INTAKE_UP_POSITION);
        }else if((potAngle < PAN_ARM_UP_LIMIT && deltaPot > 0) || potAngle < PAN_ARM_DOWN_LIMIT){
            leftServo.setPosition(PAN_SAFE_POS);
            rightServo.setPosition(PAN_SAFE_POS);
        }
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