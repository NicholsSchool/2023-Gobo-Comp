package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;

//TODO: pot conversion
//TODO: shoulder goToPos
//TODO: wrist goToPos
//TODO: climb motor

/**
 * The Arm Subsystem of the robot
 */
public class Arm implements Constants {
    private final AnalogInput pot;
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx winch;
    private final DcMotorEx wristMotor;
    private final Servo leftExtension;
    private final Servo rightExtension;
    private final Servo planeLauncher;

    /**
     * Initializes the Arm object
     *
     * @param hwMap the hardwareMap
     */
    public Arm(HardwareMap hwMap) {
        pot = hwMap.get(AnalogInput.class, "pot");

        leftShoulder = hwMap.get(DcMotorEx.class, "leftDead");
        rightShoulder = hwMap.get(DcMotorEx.class, "rightDead");

        wristMotor = hwMap.get(DcMotorEx.class, "wrist");
        wristMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wristMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wristMotor.setPositionPIDFCoefficients(WRIST_P);

        winch = hwMap.get(DcMotorEx.class, "centerDead");
        winch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtension = hwMap.get(Servo.class, "leftExtension");
        rightExtension = hwMap.get(Servo.class, "rightExtension");
        planeLauncher = hwMap.get(Servo.class, "planeLauncher");

        planeLauncher.scaleRange(PLANE_LAUNCHER_COCKED, 1.0);
    }

    /**
     * Sets the Winch Power to climb the bot
     *
     * @param power the power to spin at
     */
    public void setWinch(double power) {
        winch.setPower(power);
    }

    /**
     * Moves the arm with PID
     *
     * @param angle the arm angle in degrees, 0 is retracted
     */
    public void armGoToPos(double angle) {
        double power = SHOULDER_P * (angle - getArmAngle());
        armManualControl(power);
    }

    /**
     * Moves the arm manually
     *
     * @param power the turning power [-max speed, max_speed]
     */
    public void armManualControl(double power) {
        power = MathUtilities.clip(power, -SHOULDER_GOVERNOR, SHOULDER_GOVERNOR);
        leftShoulder.setPower(-power);
        rightShoulder.setPower(-power);
    }


    /**
     * Moves the wrist manually
     *
     * @param power the turning power [-max_speed, max_speed]
     */
    public void wristManualControl(double power) {
        power = MathUtilities.clip(power, -WRIST_GOVERNOR, WRIST_GOVERNOR);
        wristMotor.setPower(power);
    }

    /**
     * Turns the wrist to the specified encoder position using PID
     *
     * @param position the encoder target position
     */
    public void setWristPos(int position) {
        wristMotor.setTargetPosition(position);
    }

    /**
     * Fully Extend or Retract Linear Actuators on the Arm
     *
     * @param isExtending whether to extend or retract
     */
    public void setExtensionPos(boolean isExtending) {
        leftExtension.setPosition(isExtending ? 1.0 : 0.0);
        rightExtension.setPosition(isExtending ? 1.0 : 0.0);
    }

    /**
     * Move Linear Actuators on the Arm Manually
     *
     * @param position the position to extend to [0, 1]
     */
    public void setExtensionManual(double position) {
        leftExtension.setPosition(position);
        rightExtension.setPosition(position);
    }

    /**
     * Shoots the plane launcher
     *
     * @param shoot whether to shoot the plane
     */
    public void setPlaneLauncher(boolean shoot) {
        planeLauncher.setPosition(shoot ? 1.0 : PLANE_LAUNCHER_COCKED);
    }

    /**
     * ACQUIRE THE ZA
     *
     * @return the za aka the raw potentiometer voltage output
     */
    public double getPot() {
        return pot.getVoltage();
    }

    /**
     * Uses the potentiometer value and a conversion to
     * estimate the angle of the arm.
     *
     * @return the arm angle in degrees, with 0 as fully in.
     */
    public double getArmAngle() {
        double potVal = this.getPot();
        return POT_COEFF_D * Math.pow( potVal, 3 ) + POT_COEFF_C * Math.pow( potVal, 2) + POT_COEFF_B * potVal + POT_COEFF_A - 60.0;
    }

    /**
     * Return the Wrist Position
     *
     * @return the motor's encoder position
     */
    public int getWristPos() {
        return wristMotor.getCurrentPosition();
    }
}
