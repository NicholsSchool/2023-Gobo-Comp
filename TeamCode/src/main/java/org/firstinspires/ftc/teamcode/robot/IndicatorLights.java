package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * The class defining the Indicator Lights on the robot.
 * Here is a full list of
 * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">blink codes</a>.
 */
public class IndicatorLights {

    private RevBlinkinLedDriver leftBlinkin;
    private RevBlinkinLedDriver rightBlinkin;
    private RevBlinkinLedDriver.BlinkinPattern defaultPattern;

    /**
     * Initializes the IndicatorLights subsystem
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance whether we are blue alliance
     */
    public void init(HardwareMap hwMap, boolean isBlueAlliance) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "rightBlinkin");
        defaultPattern = isBlueAlliance ? RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE
                : RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
        setColour(defaultPattern);
    }

    /** Sets both left and right LED strips to a certain colour pattern.
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets left LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setLeftColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
    }

    /**
     * Sets right LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setRightColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets the default color, red or blue.
     */
    public void setDefaultColor() {
        leftBlinkin.setPattern(defaultPattern);
        rightBlinkin.setPattern(defaultPattern);
    }
}