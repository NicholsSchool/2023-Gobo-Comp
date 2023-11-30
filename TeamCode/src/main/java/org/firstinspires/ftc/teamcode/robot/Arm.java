package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: plane launcher, pot conversion, shoulder PID

/**
 * The Arm Subsystem of the robot
 */
public class Arm implements Constants {
    private AnalogInput pot;
    private DcMotorEx leftShoulder;
    private DcMotorEx rightShoulder;
    private Servo innerLeftExtension;
    private Servo innerRightExtension;
    //private Servo planeLauncher;

    /**
     * Initializes the Arm object
     *
     * @param hwMap the hardwareMap
     */
    public void init(HardwareMap hwMap) {
        pot = hwMap.get(AnalogInput.class, "pot");

        leftShoulder = hwMap.get(DcMotorEx.class, "leftDead");
        rightShoulder = hwMap.get(DcMotorEx.class, "rightDead");

        innerLeftExtension = hwMap.get(Servo.class, "innerLeftExtension");
        innerRightExtension = hwMap.get(Servo.class, "innerRightExtension");
        //planeLauncher = hwMap.get(Servo.class, "planeLauncher");
    }

    /**
     * Moves the arm manually
     *
     * @param power the turning power
     */
    public void armManualControl(double power) {
        power = Range.clip(power, -ARM_GOVERNOR, ARM_GOVERNOR);
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * Linear Actuators on the Arm
     *
     * @param isExtending whether to extend or retract
     */
    public void extensionGoTo(boolean isExtending) {
        innerLeftExtension.setPosition(isExtending ? EXTENSION_OUT_POSITION : EXTENSION_IN_POSITION);
        innerRightExtension.setPosition(isExtending ? EXTENSION_OUT_POSITION : EXTENSION_IN_POSITION);
    }

    /**
     * ACQUIRE THE ZA
     *
     * @return the za aka the potentiometer voltage output
     */
    public double getPot() {
        return pot.getVoltage();
    }
}
