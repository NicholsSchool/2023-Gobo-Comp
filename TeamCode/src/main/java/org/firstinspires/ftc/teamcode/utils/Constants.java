package org.firstinspires.ftc.teamcode.utils;

/**
 * Robot Constants
 */
public interface Constants {
    /** Blue Alliance Tag */
    boolean BLUE_ALLIANCE = true;

    /** Red Alliance Tag */
    boolean RED_ALLIANCE = false;

    /** The Default Controller Axis DeadBand */
    double DEFAULT_DEADBAND = 0.05;

    /** The Maximum Spin Speed of a drive motor in ticks/second */
    int MAX_SPIN_SPEED = 2800;

    /** The Governor for Maximum Speed as a proportion of available power */
    double OVERALL_GOVERNOR = 1.0;

    /** The Governor for Turning Speed as a proportion of available power */
    double TURNING_GOVERNOR = 0.3;

    /** The Proportional Constant for PID turning to an angle */
    double TURNING_P = 0.025;

    /** The +/- allowed error for autoAligning */
    double TURNING_ERROR = 0.5;

    /** The Feedforward Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_FF = 12.0;

    /** The Feedforward Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_FF = 12.0;

    /** The Feedforward Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_FF = 12.0;

    /** The Feedforward Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_FF = 12.0;

    /** Ticks per revolution of the dead wheels */
    int TICKS_PER_REV = 8192;

    /** Approximate Diameter of our Dead Wheels */
    double DEAD_DIAMETER = 2.5;

    /** Inches driven per encoder tick of a dead wheel */
    double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;

    /** The Multiplier to forward distance tracking */
    double FORWARD_ODOMETRY_CORRECTION = 130.0 / 134.0;

    /** The Multiplier to strafe distance tracking */
    double STRAFE_ODOMETRY_CORRECTION = 130.0 / 132.5;

    /** The Proportional Constant for PID spline */
    double SPLINE_P = 0.1;

    /** The X value of the Scoring Points */
    double SCORING_X = -48.0;

    /** The Y value of the Blue Scoring Point */
    double BLUE_SCORING_Y = 36.0;

    /** The Y value of the Red Scoring Point */
    double RED_SCORING_Y = -36.0;

    /** The X value of the Left Waypoint Points */
    double LEFT_WAYPOINT_X = -12.0;

    /** The X value of the Right Waypoint Points */
    double RIGHT_WAYPOINT_X = 36.0;

    /** The Y value of the Blue Waypoint Points */
    double BLUE_WAYPOINT_Y = 36.0;

    /** The Y value of the Red Waypoint Point */
    double RED_WAYPOINT_Y = -36.0;

    /** The X value of the Intake Points */
    double INTAKE_X = 60.0;

    /** The Y value of the Blue Intake Point */
    double BLUE_INTAKE_Y = 60.0;

    /** The Y value of the Red Intake Point */
    double RED_INTAKE_Y = -60.0;
}
