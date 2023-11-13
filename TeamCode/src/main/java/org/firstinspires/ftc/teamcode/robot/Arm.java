package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Arm Subsystem of the robot
 */
public class Arm implements Constants {
    private AnalogInput pot;

    /**
     * Initializes the Arm object
     *
     * @param hwMap the hardwareMap
     */
    public void init(HardwareMap hwMap) {
        pot = hwMap.get(AnalogInput.class, "pot");
    }

    /**
     * ACQUIRE THE ZA
     *
     * @return the raw potentiometer value
     */
    public double getPot() {
        return pot.getVoltage();
    }

}
