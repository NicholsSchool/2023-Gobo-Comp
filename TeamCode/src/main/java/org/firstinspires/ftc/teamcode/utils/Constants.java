package org.firstinspires.ftc.teamcode.utils;

/**
 * Robot Constants contained in a convenient interface
 */
public interface Constants {
    /** Blue Alliance Tag */
    boolean BLUE_ALLIANCE = true;

    /** Red Alliance Tag */
    boolean RED_ALLIANCE = false;

    /** The Default Controller Axis DeadBand */
    double DEFAULT_DEADBAND = 0.01;

    /** The Maximum Spin Speed of a drive motor in ticks/second */
    int MAX_SPIN_SPEED = 2800;

    /** The Governor for Maximum Speed as a proportion of available power */
    double OVERALL_GOVERNOR = 0.9;

    /** The Governor for Manual Turning Speed as a proportion of available power */
    double MANUAL_TURNING_GOVERNOR = 0.3;

    /** The Governor for Auto Turning Speed as a proportion of available power */
    double AUTO_TURNING_GOVERNOR = 0.3;

    /** The Proportional Constant for PID turning to an angle */
    double TURNING_P = 0.019;

    /** The +/- allowed error for autoAligning in degrees */
    double TURNING_ERROR = 0.5;

    /** The number of code loops to wait before auto-aligning */
    int LOOPS_TO_WAIT = 5;

    /** The Proportional Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_P = 7.5;

    /** The Integral Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_I = 0.0;

    /** The Dampening Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_D = 4.0;

    /** The Feedforward Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_F = 12.8;

    /** The Proportional Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_P = 7.5;

    /** The Integral Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_I = 0.0;

    /** The Dampening Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_D = 4.0;

    /** The Feedforward Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_F = 12.8;

    /** The Proportional Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_P = 7.5;

    /** The Integral Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_I = 0.0;

    /** The Dampening Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_D = 4.0;

    /** The Feedforward Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_F = 12.8;

    /** The Proportional Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_P = 7.5;

    /** The Integral Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_I = 0.0;

    /** The Dampening Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_D = 4.0;

    /** The Feedforward Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_F = 12.8;

    /** Ticks per revolution of a REV thru bore encoder */
    int TICKS_PER_REV = 8192;

    /** Approximate Diameter of our Dead Wheels */
    double DEAD_DIAMETER = 2.5;

    /** The distance between the center of the left and right dead wheels in inches */
    double ROBOT_TRACKWIDTH = 5.8;

    /** Inches driven per encoder tick of a dead wheel */
    double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;

    double DEGREES_PER_TICK = DEAD_DIAMETER * .5 * 360.0 / (TICKS_PER_REV * ROBOT_TRACKWIDTH);

    /** The Multiplier for forward distance tracking */
    double FORWARD_ODOMETRY_CORRECTION = 0.964;

    /** The Multiplier for strafe distance tracking */
    double STRAFE_ODOMETRY_CORRECTION = 0.964;

    /** THe Multiplier for heading tracking */
    double HEADING_ODOMETRY_CORRECTION = 0.9099;

    /** The Proportional Constant for PID spline */
    double SPLINE_P = 0.1;

    /** The X value of the Scoring Points */
    double SCORING_X = -36.0;

    /** The Y value of the Blue Scoring Point */
    double BLUE_SCORING_Y = -36.0;

    /** The Y value of the Red Scoring Point */
    double RED_SCORING_Y = 36.0;

    /** The X value of the Left Waypoint Points */
    double LEFT_WAYPOINT_X = -12.0;

    /** The X value of the Right Waypoint Points */
    double RIGHT_WAYPOINT_X = 36.0;

    /** The Y value of the Blue Waypoint Points */
    double BLUE_WAYPOINT_Y = 36.0;

    /** The Y value of the Red Waypoint Point */
    double RED_WAYPOINT_Y = -36.0;

    /** The X value of the Intake Points */
    double INTAKE_X = 48.0;

    /** The Y value of the Blue Intake Point */
    double BLUE_INTAKE_Y = 48.0;

    /** The Y value of the Red Intake Point */
    double RED_INTAKE_Y = -48.0;

    /** Servo position for intake actuators when up */
    double INTAKE_UP_POSITION = 0.25;

    /** Servo position for intake actuators when touching floor */
    double INTAKE_DOWN_POSITION = 0.75;
}
