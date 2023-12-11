package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: access all arm functionalities

/**
 * A teleop for testing Arm functionalities using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Arm Testing")
public class ArmTestingTeleop extends OpMode implements Constants
{
    public Arm arm;
    public static double shoulderPower;
    public static double extensionPosition;
    public static boolean launchPlane;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        arm.armManualControl(shoulderPower);
        arm.setExtensionManual(extensionPosition);
        arm.setPlaneLauncher(launchPlane);

        telemetry.addData("shoulderPower", shoulderPower);
        telemetry.addData("extensionPosition", extensionPosition);
        telemetry.addData("launchPlane", launchPlane);
        telemetry.addData("pot", arm.getPot());
        telemetry.update();
    }
}
