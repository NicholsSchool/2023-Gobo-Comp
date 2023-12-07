package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * A teleop for testing Vision functionalities
 */
@Config
@TeleOp(name="Vision Testing")
public class VisionTestingTeleop extends OpMode implements Constants
{
    private Vision vision;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    @Override
    public void loop() {
        double[] pose = vision.update();
        telemetry.addData("num detections", vision.getNumDetections());
        telemetry.addData("X", pose != null ? pose[0] : "null");
        telemetry.addData("Y", pose != null ? pose[1] : "null");
        telemetry.addData("Theta", pose != null ? pose[2] : "null");
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.close();
    }
}
