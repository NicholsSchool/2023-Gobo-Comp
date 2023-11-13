package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.io.PrintWriter;

@Config
@TeleOp(name="Tuning Opmode")
public class DriveMotorTuningTeleop extends OpMode implements Constants
{
    private final ElapsedTime runtime = new ElapsedTime();
    public RobotContainer robotContainer;
    public static double lbp = BACK_LEFT_P;
    public static double lbi = BACK_LEFT_I;
    public static double lbd = BACK_LEFT_D;
    public static double lbf = BACK_LEFT_F;
    public static double rbp = BACK_RIGHT_P;
    public static double rbi = BACK_RIGHT_I;
    public static double rbd = BACK_RIGHT_D;
    public static double rbf = BACK_RIGHT_F;
    public static double lfp = FRONT_LEFT_P;
    public static double lfi = FRONT_LEFT_I;
    public static double lfd = FRONT_LEFT_D;
    public static double lff = FRONT_LEFT_F;
    public static double rfp = FRONT_RIGHT_P;
    public static double rfi = FRONT_RIGHT_I;
    public static double rfd = FRONT_RIGHT_D;
    public static double rff = FRONT_RIGHT_F;
    public static double targetMotorVelocity;

    @Override
    public void init() {
        robotContainer = new RobotContainer();
        robotContainer.init(hardwareMap, true, 0, 0);
        robotContainer.drivetrain.backLeft.setVelocityPIDFCoefficients(lbp, lbi, lbd, lbf);
        robotContainer.drivetrain.backRight.setVelocityPIDFCoefficients(rbp, rbi,rbd, rbf);
        robotContainer.drivetrain.frontLeft.setVelocityPIDFCoefficients(lfp, lfi, lfd, lff);
        robotContainer.drivetrain.frontRight.setVelocityPIDFCoefficients(rfp, rfi, rfd, rff);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if(runtime.time() > 5.0) {
            runtime.reset();
            targetMotorVelocity *= -1;
        }
        robotContainer.drivetrain.backLeft.setVelocityPIDFCoefficients(lbp, lbi, lbd, lbf);
        robotContainer.drivetrain.backRight.setVelocityPIDFCoefficients(rbp, rbi,rbd, rbf);
        robotContainer.drivetrain.frontLeft.setVelocityPIDFCoefficients(lfp, lfi, lfd, lff);
        robotContainer.drivetrain.frontRight.setVelocityPIDFCoefficients(rfp, rfi, rfd, rff);

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
