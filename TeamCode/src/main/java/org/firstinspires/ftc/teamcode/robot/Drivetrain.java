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
 * TODO: feedforward tuning
 * TODO: odometry tuning
 */
public class Drivetrain implements Constants {
    private BHI260IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, leftDead, rightDead, centerDead;
    private int previousLeft, previousRight, previousCenter;
    private double x, y, headingOffset;
    private boolean isBlueAlliance;

    public void init(HardwareMap hwMap, boolean isBlueAlliance, double x, double y)
    {
        // Initialize Variables
        this.isBlueAlliance = isBlueAlliance;
        this.previousLeft = 0;
        this.previousRight = 0;
        this.previousCenter = 0;
        this.x = x;
        this.y = y;
        this.headingOffset = 0.0;

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
        centerDead.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Zero Power Behavior
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // Set Motor FF Coefficients
        backLeft.setVelocityPIDFCoefficients(0, 0, 0, BACK_LEFT_FF);
        backRight.setVelocityPIDFCoefficients(0, 0, 0, BACK_RIGHT_FF);
        frontLeft.setVelocityPIDFCoefficients(0, 0, 0, FRONT_LEFT_FF);
        frontRight.setVelocityPIDFCoefficients(0, 0, 0, FRONT_RIGHT_FF);
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
     */
    public void drive(double power, double angle, double turn, boolean autoAlign)
    {
        double desiredAngle = 0.0; //TODO: desiredAngle calculations in this class, not in the teleop
        double heading = getFieldHeading();
        power = Range.clip(power, -DRIVING_GOVERNOR, DRIVING_GOVERNOR);

        if(autoAlign)
            turn = Calculator.turnToAngle(heading, desiredAngle);
        else
            turn = Range.clip(turn, -TURNING_GOVERNOR, TURNING_GOVERNOR);

        double corner1 = power * Math.sin(Math.toRadians(angle - 45.0 + 90.0 - heading));
        double corner2 = power * Math.sin(Math.toRadians(angle + 45.0 + 90.0 - heading));
        //TODO: include headingOffset in corner calculations for resseting field oriented
        backLeft.setVelocity((corner1 + turn) * MAX_SPIN_SPEED);
        backRight.setVelocity((corner2 - turn) * MAX_SPIN_SPEED);
        frontLeft.setVelocity((corner2 + turn) * MAX_SPIN_SPEED);
        frontRight.setVelocity((corner1 - turn) * MAX_SPIN_SPEED);
    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees [-180, 180)
     */
    public double getFieldHeading()
    {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (isBlueAlliance)
            return Calculator.addAngles(angle, 90.0);
        else
            return Calculator.addAngles(angle, -90.0);
    }

    /**
     * Set Field Oriented forward to the robot's heading by offsetting the IMU yaw
     */
    public void resetFieldOriented() {
        headingOffset = getFieldHeading();
    }

    /**
     * Update the robot's odometry, call at the start of each loop() cycle
     */
    public void updateOdometry() {
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