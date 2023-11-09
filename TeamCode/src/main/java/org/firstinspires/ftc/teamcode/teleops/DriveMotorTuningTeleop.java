package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Config
@TeleOp(name="Tuning Opmode")
public class DriveMotorTuningTeleop extends OpMode
{
    public RobotContainer robotContainer;
    public static double p, i, d, f;
    public static double targetMotorVelocity;

    @Override
    public void init() {
        robotContainer = new RobotContainer();
        robotContainer.init(hardwareMap, true);
        robotContainer.drivetrain.backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.backRight.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.frontRight.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robotContainer.drivetrain.backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.backRight.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        robotContainer.drivetrain.frontRight.setVelocityPIDFCoefficients(p, i, d, f);

        robotContainer.drivetrain.driveTest(targetMotorVelocity / 2800.0);

        double backLeftVel = robotContainer.drivetrain.backLeft.getVelocity();
        double backRightVel = robotContainer.drivetrain.backRight.getVelocity();
        double frontLeftVel = robotContainer.drivetrain.frontLeft.getVelocity();
        double frontRightVel = robotContainer.drivetrain.frontRight.getVelocity();

        telemetry.addData("back left vel", backLeftVel);
        telemetry.addData("back right vel", backRightVel);
        telemetry.addData("front left vel", frontLeftVel);
        telemetry.addData("front right vel", frontRightVel);
        telemetry.addData("target vel", targetMotorVelocity);
        telemetry.update();
    }
}
