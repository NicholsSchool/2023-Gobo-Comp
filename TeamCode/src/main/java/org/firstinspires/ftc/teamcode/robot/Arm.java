package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Arm Subsystem of the robot TODO:check if re-instantiation breaks anything
 */
public class Arm implements Constants {
    private AnalogSensor pot;

    /**
     * Initializes the Arm object
     *
     * @param hwMap the hardwareMap
     */
    public void init(HardwareMap hwMap) {
        pot = hwMap.get(AnalogSensor.class, "pot");

    }

    /**
     * ACQUIRE THE ZA
     *
     * @return the za
     */
    public double getPot() {
        return pot.readRawVoltage();
    }

}
