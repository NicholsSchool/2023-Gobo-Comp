package org.firstinspires.ftc.teamcode.utils;

/**
 * Robot Constants
 */
public interface Constants {
    /** Blue Alliance Tag */
    public static final boolean BLUE_ALLIANCE = true;

    /** Red Alliance Tag */
    public static final boolean RED_ALLIANCE = false;

    /** The Default Controller Axis DeadBand */
    public static final double DEFAULT_DEADBAND = 0.05;

    /** The Maximum Spin Speed of a drive motor in ticks/second */
    public static final int MAX_SPIN_SPEED = 2800;

    /** The Governor for Driving Speed as a proportion of available power */
    public static final double DRIVING_GOVERNOR = 0.75;

    /** The Governor for Turning Speed as a proportion of available power */
    public static final double TURNING_GOVERNOR = 0.25;

    /** The Proportional Constant for PID turning to an angle */
    public static final double TURNING_P = 0.1;

    /** The Feedforward Coefficient for the Back Left Drive Motor */
    public static final double BACK_LEFT_FF = 10.0;

    /** The Feedforward Coefficient for the Back Right Drive Motor */
    public static final double BACK_RIGHT_FF = 10.0;

    /** The Feedforward Coefficient for the Front Left Drive Motor */
    public static final double FRONT_LEFT_FF = 10.0;

    /** The Feedforward Coefficient for the Front Right Drive Motor */
    public static final double FRONT_RIGHT_FF = 10.0;

    /** Ticks per revolution of the dead wheels */
    public static final int TICKS_PER_REV = 8192;

    /** Approximate Diameter of our Dead Wheels */
    public static final double DEAD_DIAMETER = 2.5;

    /** Inches driven per encoder tick of a dead wheel */
    public static final double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;

    /** The Multiplier to forward distance tracking */
    public static final double FORWARD_ODOMETRY_CORRECTION = 1.0;

    /** The Multiplier to strafe distance tracking */
    public static final double STRAFE_ODOMETRY_CORRECTION = 1.0;

    /** Servo position for intake actuators when vertical */
    public static final double INTAKE_UP_POSITION = 0.2;

    /** Servo position for intake actuators when touching floor */
    public static final double INTAKE_DOWN_POSITION = 1;
}
