package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: plane launcher, pot conversion, then shoulder PID

/**
 * The Arm Subsystem of the robot
 */
public class Arm implements Constants {
    private final AnalogInput pot;
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final Servo leftExtension;
    private final Servo rightExtension;
    public double previousPot;
    public double deltaPot;
    //TODO: private Servo planeLauncher;

    /**
     * Initializes the Arm object
     *
     * @param hwMap the hardwareMap
     */
    public Arm(HardwareMap hwMap) {
        pot = hwMap.get(AnalogInput.class, "pot");

        leftShoulder = hwMap.get(DcMotorEx.class, "leftDead");
        rightShoulder = hwMap.get(DcMotorEx.class, "rightDead");

        leftExtension = hwMap.get(Servo.class, "leftExtension");
        rightExtension = hwMap.get(Servo.class, "rightExtension");

        this.previousPot = 0;
        //planeLauncher = hwMap.get(Servo.class, "planeLauncher");
    }

    /**
     * Moves the arm manually
     *
     * @param power the turning power
     */
    public void armManualControl(double power) {
        power = Range.clip(power, -SHOULDER_GOVERNOR, SHOULDER_GOVERNOR);
        leftShoulder.setPower(-power);
        rightShoulder.setPower(power);
    }

    /**
     * Move Linear Actuators on the Arm
     *
     * @param isExtending whether to extend or retract
     */
    public void setExtensionPos(boolean isExtending) {
        leftExtension.setPosition(isExtending ? EXTENSION_OUT_POSITION : EXTENSION_IN_POSITION);
        rightExtension.setPosition(isExtending ? EXTENSION_OUT_POSITION : EXTENSION_IN_POSITION);
    }

    /**
     * ACQUIRE THE ZA
     *
     * @return the za aka the potentiometer voltage output
     */
    public double getPot() {
        return pot.getVoltage();
    }

    public double potToAngle(){
        double angle = Math.log(2.41854 / getPot() - 1) / 0.0100245 + 415.482;
        return angle;
    }
    public void deltaPot(){
        double pot = getPot();
        deltaPot = 100 * (pot - previousPot);
        previousPot = pot;
    }

}
