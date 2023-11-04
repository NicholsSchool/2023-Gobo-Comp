package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import java.util.concurrent.TimeUnit;



public class IndicatorLights {
    
    public RevBlinkinLedDriver leftBlinkin;
    public RevBlinkinLedDriver rightBlinkin;

    public void init(HardwareMap hwMap) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "leftBlinkin");
    }

    /** Sets both left and right LED strips to a certain colour pattern.
     * 
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     * 
     * Possible patterns: https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html
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
     * Possible patterns: https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html
     */
    public void setLeftColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
    }

    /**
     * Sets right LED strip to a certain colour pattern
     * 
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     * 
     * Possible patterns: https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html
     */
    public void setRightColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        rightBlinkin.setPattern(pattern);
    }
}
