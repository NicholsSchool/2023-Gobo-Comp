package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

/**
 * A Math Utilities Class for the Robot.
 * Also contains methods for advanced autonomous pathing.
 * The premise of these methods is to define piecewise
 * parabolas to waypoints and have the robot follow
 * those curves.
 */
public class Calculator implements Constants {
    /**
     * Adds two angles that are measured in degrees
     *
     * @param angle1 the first angle
     * @param angle2 the second angle
     * @return the sum in the range [-180, 180)
     */
    public static double addAngles(double angle1, double angle2) {
        double sum = angle1 + angle2;
        while (sum >= 180.0) sum -= 360.0;
        while (sum < -180) sum += 360.0;
        return sum;
    }

    /**
     * Spins the robot anchor-less to a given heading smoothly using PID
     *
     * @param heading        the current robot heading
     * @param desiredHeading the desired robot heading
     * @return the turning speed as a proportion
     */
    public static double turnToAngle(double heading, double desiredHeading) {
        double error = desiredHeading - heading;
        return Range.clip(error * TURNING_P, -TURNING_GOVERNOR, TURNING_GOVERNOR);
    }

    /**
     * With the robot at (rx, ry), calculates the drive angle of the robot
     * in order to arrive at the waypoint (wx, wy) with the robot's vector
     * approaching a horizontal value, 0 or -180 smoothly
     *
     * @param rx the robot's x coordinate
     * @param ry the robot's y coordinate
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @return the drive angle in degrees [-180, 180)
     */
    public static double angleToVertex(double rx, double ry, double wx, double wy) {
        if (rx == wx && ry == wy) return 0.0;
        double angle = Math.toDegrees(Math.atan2(2.0 * (ry - wy), rx - wx));
        if (rx > wx) return addAngles(angle, 180.0);
        return addAngles(angle, 0.0);
    }

    /**
     * With the robot at (rx, ry), calculates the drive angle of the robot
     * in order to arrive at the waypoint (wx, wy) with the robot's vector
     * leaving a horizontal value, 0 or -180 smoothly
     *
     * @param rx the robot's x coordinate
     * @param ry the robot's y coordinate
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param h  the x value of the previous waypoint
     * @return the drive angle in degrees [-180, 180)
     */
    public static double angleFromVertex(double rx, double ry, double wx, double wy, double h) {
        if (rx == h || Math.abs(rx - h) == Math.abs(wx - h)) return 0.0;
        double robotDiff = Math.pow(rx - h, 2);
        double waypointDiff = Math.pow(wx - h, 2);
        double k = (wy * robotDiff - ry * waypointDiff) / (robotDiff - waypointDiff);
        double angle = Math.toDegrees(Math.atan2(2.0 * (ry - k), rx - h));
        if (rx > wx) return addAngles(angle, 180.0);
        return addAngles(angle, 0.0);
    }
}
