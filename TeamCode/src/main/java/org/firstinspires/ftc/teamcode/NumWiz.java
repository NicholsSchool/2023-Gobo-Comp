package org.firstinspires.ftc.teamcode;

/**
 * Performs Calculations Necessary for the Robot
 */
public class NumWiz {
    /**
     * Adds two angles measured in degrees
     * @param angle1 the first angle
     * @param angle2 the second angle
     * @return the sum in the range [-180, 180)
     */
    public static double addAngles( double angle1, double angle2 )
    {
        double sum = angle1 + angle2;
        while( sum >= 180.0 )
            sum -= 360.0;
        while( sum < -180 )
            sum += 360.0;
        return sum;
    }

    /**
     * Calculates the drive angle of the robot in order to arrive at the waypoint (wx, wy)
     * @param rx the robot's x coordinate
     * @param ry the robot's y coordinate
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @return the drive angle in degrees [-180, 180)
     */
    public static double driveToVertex( double rx, double ry, double wx, double wy )
    {
        return Math.toDegrees( Math.atan2( 2.0 * ( ry - wy ), rx - wx ) );
    }

    /**
     * Calculates the drive angle of the robot in order to arrive at the waypoint (wx, wy)
     * @param rx the robot's x coordinate
     * @param ry the robot's y coordinate
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param h the x value of the previous waypoint
     * @return the drive angle in degrees [-180, 180)
     */
    public static double driveFromVertex( double rx, double ry, double wx, double wy, double h )
    {

        double robotDiff = Math.pow( rx - h, 2 );
        double waypointDiff = Math.pow( wx - h, 2 );
        if( rx == wx ) {
            if ( ry >= wy )
                return -90.0;
            else
                return 90.0;
        }
        double k = ( wy * robotDiff - ry * waypointDiff ) / ( robotDiff - waypointDiff );
        return Math.toDegrees( Math.atan2( 2.0 * ( ry - k ), rx - h ) );
    }
}
