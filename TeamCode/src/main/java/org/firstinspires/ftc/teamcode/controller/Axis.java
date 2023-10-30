package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The class defining Axes on a GameController
 */
public class Axis implements Constants {
    private double value;
    private double previousValue;
    private final double deadBand;

    /**
     * Creates an Axis Object with the default deadBand
     */
    public Axis() {
        this(DEFAULT_DEADBAND);
    }

    /**
     * Creates an Axis Object with a specified deadBand
     * @param deadBand the desired deadBand
     */
    public Axis(double deadBand) {
        this.value = 0.0;
        this.previousValue = 0.0;
        this.deadBand = deadBand;
    }

    /**
     * The current value of the axis
     *
     * @return the value in the range [-1, 1]
     */
    public double get() {
        return value;
    }

    /**
     * Whether the axis value just changed
     *
     * @return true if the value just changed, false otherwise
     */
    public boolean valueJustChanged() {
        return value != previousValue;
    }

    /**
     * Updates the axis's current and previous values
     *
     * @param newValue the value to set the current value to
     */
    public void updateStates(double newValue) {
        previousValue = value;
        value = applyDeadBand(newValue);
    }

    private double applyDeadBand(double value) {
        return Math.abs(value) >= deadBand ? value : 0.0;
    }

    /**
     * Return the current value, for telemetry
     *
     * @return the axis's value as String
     */
    @NonNull
    public String toString() {
        return String.valueOf(get());
    }
}
