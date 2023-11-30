package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.utils.Constants;

@Config
@TeleOp(name="Arm Manual Testing")
public class ArmTestingTeleop extends OpMode implements Constants
{
    public Arm arm;
    public static double shoulderPower;
    public static boolean extend;

    @Override
    public void init() {
        arm = new Arm();
        arm.init(hardwareMap);
        shoulderPower = 0.0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        arm.armManualControl(shoulderPower);
        arm.extensionGoTo(extend);

        telemetry.addData("shoulderPower", shoulderPower);
        telemetry.addData("extending", extend);
        telemetry.addData("pot", arm.getPot());
        telemetry.update();
    }
}
