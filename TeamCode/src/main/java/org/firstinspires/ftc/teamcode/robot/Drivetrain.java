package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Calculator;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The robot drivetrain
 * TODO: reset imu method, for field oriented
 * TODO: check directions of motors and dead wheels
 * TODO: change drive and drivetest to do setVelocity()
 * TODO: feedforward tuning
 * TODO: odometry tuning
 */
public class Drivetrain implements Constants {
    private BHI260IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, leftDead, rightDead, centerDead;
    private int leftTicks, rightTicks, strafeTicks;
    private double x, y;
    private boolean isBlueAlliance;

    public void init(HardwareMap hwMap, boolean isBlueAlliance, double x, double y)
    {
        // Initialize Variables
        this.isBlueAlliance = isBlueAlliance;
        this.leftTicks = 0;
        this.rightTicks = 0;
        this.strafeTicks = 0;
        this.x = x;
        this.y = y;

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
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
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
     * A method used to tune FeedForward Values, spins motors at equal power
     *
     * @param power the power to spin motors at
     */
    public void driveTest(double power) //TODO: change this
    {
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * Drives the robot field-oriented
     *
     * @param power the driving power
     * @param angle the angle to drive at in degrees
     * @param turn the turning power
     */
//    public void drive(double power, double angle, double turn, boolean autoAlign, double desiredAngle)
//    { //TODO: change this, lots of changes
//        turn = Range.clip(turn, -TURN_LIMIT, TURN_LIMIT);
//
//        double heading = this.getFieldHeading();
//        if(autoAlign)
//            turn = this.autoAlign(desiredAngle, heading);
//
//        power = Range.clip(power, -MAX_LIMIT + Math.abs(turn), MAX_LIMIT - Math.abs(turn));
//
//        double corner1 = power * Math.sin(Math.toRadians(angle - 45.0 + 90 - heading));
//        double corner2 = power * Math.sin(Math.toRadians(angle + 45.0 + 90 - heading));
//
//        backLeft.setPower(corner1 + turn);
//        backRight.setPower(corner2 - turn);
//        frontLeft.setPower(corner2 + turn);
//        frontRight.setPower(corner1 - turn);
//    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees [-180, 180)
     */
    public double getFieldHeading()
    {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (isBlueAlliance)
            return Calculator.addAngles(angle, 0.0);
        else
            return Calculator.addAngles(angle, -180.0);
    }

    /** //TODO: can this be called at the start of each loop?
     * Update the robot's odometry, call at the end of each loop() before updateEncoderPositions()
     */
//    public void updateCoordinates() {
//        double heading = Math.toRadians(this.getFieldHeading());
//        double deltaX = (centerDead.getCurrentPosition() - strafeTicks) * INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
//        double deltaY = ( (leftDead.getCurrentPosition() - leftTicks ) +
//                ( rightDead.getCurrentPosition() - rightTicks ) ) * .5 * INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;
//
//        y += -deltaX * Math.cos(heading) + deltaY * Math.sin(heading);
//        x += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
//    }

    /**
     * Updates encoder positions, call at the end of every loop
     */
    public void updateEncoderPositions() {

        leftTicks = leftDead.getCurrentPosition();
        rightTicks = rightDead.getCurrentPosition();
        strafeTicks = centerDead.getCurrentPosition();
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
     * @return the dead wheel encoder values
     */
    public double[] getPositions()
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
        return new double[]{this.x, this.y};
    }
}