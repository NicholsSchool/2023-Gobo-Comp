package org.firstinspires.ftc.teamcode.utils;

/**
 * Robot Constants
 */
public interface Constants {
    /** The Default Controller Axis DeadBand */
    public static final double DEFAULT_DEADBAND = 0.05;

    /** The Maximum Spin Speed of a drive motor in ticks/second */
    public static final int MAX_SPIN_SPEED = 2800;

    /** The Governor for Turning Speed as a proportion of available power */
    public static final double TURNING_GOVERNOR = 0.25;

    /** The Proportional Constant for PID turning to an angle */
    public static final double TURNING_P = 0.1;
}