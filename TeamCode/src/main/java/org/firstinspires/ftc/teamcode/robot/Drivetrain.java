package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Calculator;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The robot drivetrain
 */
public class Drivetrain implements Constants {
    //TODO: toubleshoot spline problem and any problem for red alliance
    //TODO: next step is april tag localization
    private BHI260IMU imu;
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, leftDead, rightDead, centerDead;
    private int previousLeft, previousRight, previousCenter;
    private double x, y, heading, headingOffset, desiredHeading;
    private boolean alliance;

    /**
     * Initializes the Drivetrain object
     *
     * @param hwMap the hardwareMap
     * @param alliance true for blue, false for red
     * @param x the starting x coordinate
     * @param y the starting y coordinate
     */
    public void init(HardwareMap hwMap, boolean alliance, double x, double y)
    {
        // Initialize Variables
        this.alliance = alliance;
        this.previousLeft = 0;
        this.previousRight = 0;
        this.previousCenter = 0;
        this.x = x;
        this.y = y;
        this.heading = alliance ? 90.0 : -90.0;
        this.headingOffset = 0.0;
        this.desiredHeading = heading;

        // Instantiating IMU Parameters, setting angleUnit...
        BHI260IMU.Parameters params = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get(BHI260IMU.class, "imu");
        imu.initialize(params);
        imu.resetYaw(); //Don't do this for actual matches

        // Initialize Motors
        backLeft = hwMap.get(DcMotorEx.class, "backLeftDrive");
        backRight = hwMap.get(DcMotorEx.class, "backRightDrive");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRight = hwMap.get(DcMotorEx.class, "frontRightDrive");
        leftDead = hwMap.get(DcMotorEx.class, "leftDead");
        rightDead = hwMap.get(DcMotorEx.class, "rightDead");
        centerDead = hwMap.get(DcMotorEx.class, "centerDead");

        // Set Motor Directions
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDead.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDead.setDirection(DcMotorSimple.Direction.REVERSE);
        centerDead.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDead.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDead.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerDead.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop and Reset Encoders
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motor RunModes
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDead.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDead.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDead.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Motor PIDF Coefficients
        backLeft.setVelocityPIDFCoefficients(BACK_LEFT_P, BACK_LEFT_I, BACK_LEFT_D, BACK_LEFT_F);
        backRight.setVelocityPIDFCoefficients(BACK_RIGHT_P, BACK_RIGHT_I, BACK_RIGHT_D, BACK_RIGHT_F);
        frontLeft.setVelocityPIDFCoefficients(FRONT_LEFT_P, FRONT_LEFT_I, FRONT_LEFT_D, FRONT_LEFT_F);
        frontRight.setVelocityPIDFCoefficients(FRONT_RIGHT_P, FRONT_RIGHT_I, FRONT_RIGHT_D, FRONT_RIGHT_F);
    }

    /**
     * A testing and tuning method, spins motors at equal power
     *
     * @param power the power proportion to spin motors at
     */
    public void driveTest(double power)
    {
        backLeft.setVelocity(power * MAX_SPIN_SPEED);
        backRight.setVelocity(power * MAX_SPIN_SPEED);
        frontLeft.setVelocity(power * MAX_SPIN_SPEED);
        frontRight.setVelocity(power * MAX_SPIN_SPEED);
    }

    /**
     * Drives the robot field-oriented
     *
     * @param power the driving power
     * @param angle the angle to drive at in degrees
     * @param turn the turning power
     * @param autoAlign whether to autoAlign
     * @param fieldOriented whether to drive field oriented
     */
    public void drive(double power, double angle, double turn, boolean autoAlign, boolean fieldOriented)
    {
        if(autoAlign && fieldOriented)
            turn = turnToAngle();
        else
            turn = Range.clip(turn, -TURNING_GOVERNOR, TURNING_GOVERNOR);

        power = Range.clip(power, turn - OVERALL_GOVERNOR, OVERALL_GOVERNOR - turn);

        double corner1;
        double corner2;
        if(fieldOriented) {
            corner1 = power * Math.sin(Math.toRadians(Calculator.addAngles(angle, -45.0 + 90.0 - heading)));
            corner2 = power * Math.sin(Math.toRadians(Calculator.addAngles(angle, 45.0 + 90.0 - heading)));
        }
        else {
            corner1 = power * Math.sin(Math.toRadians(Calculator.addAngles(angle, -45.0)));
            corner2 = power * Math.sin(Math.toRadians(Calculator.addAngles(angle, 45.0)));
        }

        backLeft.setVelocity((corner1 + turn) * MAX_SPIN_SPEED);
        backRight.setVelocity((corner2 - turn) * MAX_SPIN_SPEED);
        frontLeft.setVelocity((corner2 + turn) * MAX_SPIN_SPEED);
        frontRight.setVelocity((corner1 - turn) * MAX_SPIN_SPEED);
    }

    /**
     * Spins the robot anchor-less to a given heading smoothly using PID
     *
     * @return the turning speed as a proportion
     */
    public double turnToAngle() {
        double error = Calculator.addAngles(heading, -desiredHeading);
        if(Math.abs(error) >= TURNING_ERROR)
            return Range.clip(error * TURNING_P, -TURNING_GOVERNOR, TURNING_GOVERNOR);
        return 0.0;
    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees [-180, 180)
     */
    public double getFieldHeading() {
        return heading;
    }

    /**
     * Get the Raw IMU Heading of the Robot
     *
     * @return the heading in degrees [-180, 180)
     */
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Set Field Oriented forward to the robot's heading by offsetting the IMU yaw
     */
    public void setHeadingOffset() {
        headingOffset = getRawHeading();
    }

    /**
     * Sets the heading to auto-align to
     * @param desiredHeading the heading in degrees [-180, 180)
     */
    public void setDesiredHeading(double desiredHeading) {
        this.desiredHeading = desiredHeading;
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Intake
     */
    public void splineToIntake(double turn, boolean autoAlign) {
        double power = Range.clip(SPLINE_P * Math.sqrt(
                Math.pow(INTAKE_X - x, 2) + Math.pow(alliance ? BLUE_INTAKE_Y - y : RED_INTAKE_Y - y, 2)), -1.0, 1.0);

        if(x <= LEFT_WAYPOINT_X)
            drive(power, angleToVertex(x, y, LEFT_WAYPOINT_X, alliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true), turn, autoAlign, true);
        else if(x <= RIGHT_WAYPOINT_X)
            drive(power, angleToVertex(x, y, RIGHT_WAYPOINT_X, alliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true), turn, autoAlign, true);
        else
            drive(power, angleFromVertex(x, y, INTAKE_X, alliance ? BLUE_INTAKE_Y : RED_INTAKE_Y, RIGHT_WAYPOINT_X, true), turn, autoAlign, true);
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Backstage
     */
    public void splineToScoring(double turn, boolean autoAlign) {
        double power = Range.clip(SPLINE_P * Math.sqrt(
                Math.pow(SCORING_X - x, 2) + Math.pow(alliance ? BLUE_SCORING_Y - y : RED_SCORING_Y - y, 2)), -1.0, 1.0);

        if(x >= RIGHT_WAYPOINT_X)
            drive(power, angleToVertex(x, y, RIGHT_WAYPOINT_X, alliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, false), turn, autoAlign, true);
        else if(x >= LEFT_WAYPOINT_X)
            drive(power, angleToVertex(x, y, LEFT_WAYPOINT_X, alliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, false), turn, autoAlign, true);
        else
            drive(power, angleFromVertex(x, y, SCORING_X, alliance ? BLUE_SCORING_Y : RED_SCORING_Y, LEFT_WAYPOINT_X, false), turn, autoAlign, true);
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
    public double angleToVertex(double rx, double ry, double wx, double wy, boolean toIntake) {
        if(rx == wx && ry == wy)
            return toIntake ? 0.0 : -180.0;
        double angle = Math.toDegrees(Math.atan2(2.0 * (ry - wy), rx - wx));
        if(toIntake)
            return Calculator.addAngles(angle, -180.0);
        return Calculator.addAngles(angle, 0.0);
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
        if(toIntake == Math.abs(rx) > Math.abs(wx))
            return Calculator.addAngles(angle, -180.0);
        return Calculator.addAngles(angle, 0.0);
    }

    /**
     * Update the robot's pose using odometry and April Tags.
     * Call at the start of each loop() cycle
     */
    public void updatePose() {
        double angle = getRawHeading();
        if(alliance)
            heading = Calculator.addAngles(angle, 90.0 - headingOffset);
        else
            heading = Calculator.addAngles(angle, -90.0 - headingOffset);

        int currentLeft = leftDead.getCurrentPosition();
        int currentRight = rightDead.getCurrentPosition();
        int currentCenter = centerDead.getCurrentPosition();
        double heading = Math.toRadians(getFieldHeading());

        double deltaX = (currentCenter - previousCenter) * INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
        double deltaY = ( (currentLeft - previousLeft ) +
                ( currentRight - previousRight ) ) * .5 * INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;

        y += -deltaX * Math.cos(heading) + deltaY * Math.sin(heading);
        x += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        previousLeft = currentLeft;
        previousRight = currentRight;
        previousCenter = currentCenter;
    }

    /**
     * Get Motor Positions for telemetry
     *
     * @return in order: backLeft, backRight, frontLeft, frontRight positions
     */
    public double[] getMotorPositions()
    {
        return new double[]{
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition()
        };
    }

    /**
     * Get Motor Velocities for telemetry
     *
     * @return in order: backLeft, backRight, frontLeft, frontRight velocities
     */
    public double[] getMotorVelocities()
    {
        return new double[]{
                backLeft.getVelocity(),
                backRight.getVelocity(),
                frontLeft.getVelocity(),
                frontRight.getVelocity()
        };
    }

    /**
     * Get the dead wheel position values for telemetry
     *
     * @return the dead wheel encoder values in the order:
     * left, right, center
     */
    public double[] getOdometryPositions()
    {
        return new double[]{
                leftDead.getCurrentPosition(),
                rightDead.getCurrentPosition(),
                centerDead.getCurrentPosition()
        };
    }

    /**
     * Get the robot coordinates for telemetry
     *
     * @return the x and y value of the center of the bot
     */
    public double[] getXY() {
        return new double[]{x, y};
    }
}