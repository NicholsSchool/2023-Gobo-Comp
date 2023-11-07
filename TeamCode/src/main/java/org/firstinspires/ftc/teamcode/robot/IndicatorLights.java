package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * The class defining the Blinkins on the robot
 */
public class IndicatorLights {

    private RevBlinkinLedDriver leftBlinkin;
    private RevBlinkinLedDriver rightBlinkin;

    public void init(HardwareMap hwMap) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "rightBlinkin");
    }

    /** Sets both left and right LED strips to a certain colour pattern.
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     *
     * Possible patterns:
     * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">...</a>
     */
    public void setColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets left LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     *
     * Possible patterns:
     * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">...</a>
     */
    public void setLeftColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
    }

    /**
     * Sets right LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     *
     * Possible patterns:
     * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">...</a>
     */
    public void setRightColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        rightBlinkin.setPattern(pattern);
    }
}