package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Constants {
    
    private Servo leftServo;
    private Servo rightServo;

    /** Initializes pan servos (linear actuators)
     *  MUST BE CALLED DURING TELEOP INIT OR ELSE
    */
    public void init(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "leftServo");
        rightServo = hwMap.get(Servo.class, "rightServo");
    }

    /** Sets the pan to the vertical position */
    public void panUp() {
        leftServo.setPosition(INTAKE_UP_POSITION);
        rightServo.setPosition(INTAKE_UP_POSITION);
    }

    /** Sets the pan to the lowest position (to touch the ground) */
    public void panDown() {
        leftServo.setPosition(INTAKE_DOWN_POSITION);
        rightServo.setPosition(INTAKE_DOWN_POSITION);
    }

    /** Sets pan actuators to a custom position
     * @param position The position to set the pan actuators to
     */
    public void panToPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

}