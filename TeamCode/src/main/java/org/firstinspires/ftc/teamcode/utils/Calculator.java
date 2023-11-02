package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

/**
 * A Math Utilities Class for the Robot.
 * Also contains methods for advanced autonomous pathing.
 * The premise of these methods is to define piecewise
 * parabolas to waypoints and have the robot follow
 * those curves using the arc-tangent of the derivative.
 */
public class Calculator implements Constants { //TODO: move methods with only 1 usage class
    /**
     * Adds two angles that are measured in degrees
     *
     * @param angle1 the first angle
     * @param angle2 the second angle
     * @return the sum in the range [-180, 180)
     */
    public static double addAngles(double angle1, double angle2) {
        double sum = angle1 + angle2;
        while(sum >= 180.0)
            sum -= 360.0;
        while(sum < -180)
            sum += 360.0;
        return sum;
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
     * @param toIntake whether the robot is going to the intake
     * @return the drive angle in degrees [-180, 180)
     */
    public static double angleToVertex(double rx, double ry, double wx, double wy, boolean toIntake) {
        if(rx == wx && ry == wy)
            return toIntake ? 0.0 : -180.0;
        double angle = Math.toDegrees(Math.atan2(2.0 * (ry - wy), rx - wx));
        if(toIntake)
            return addAngles(angle, -180.0);
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
     * @param toIntake whether the robot is going to the intake
     * @return the drive angle in degrees [-180, 180)
     */
    public static double angleFromVertex(double rx, double ry, double wx, double wy, double h, boolean toIntake) {
        double robotDiff = Math.pow(rx - h, 2);
        double waypointDiff = Math.pow(wx - h, 2);
        if(rx == h || robotDiff == waypointDiff)
            return toIntake ? 0.0 : -180.0;
        double k = (wy * robotDiff - ry * waypointDiff) / (robotDiff - waypointDiff);
        double angle = Math.toDegrees(Math.atan2(2.0 * (ry - k), rx - h));
        if(toIntake)
            return addAngles(angle, -180.0);
        return addAngles(angle, 0.0);
    }
}
