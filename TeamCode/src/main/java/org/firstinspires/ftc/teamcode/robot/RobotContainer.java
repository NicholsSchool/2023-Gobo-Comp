package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * The Robot Class that contains all subsystems
 */
public class RobotContainer {
    public boolean alliance;
    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;
    public IndicatorLights blinkin;

    public void init(HardwareMap hwMap, boolean alliance, double x, double y) {
        this.alliance = alliance;
        drivetrain = new Drivetrain();
        intake = new Intake();
        arm = new Arm();
        blinkin = new IndicatorLights();
        drivetrain.init(hwMap, alliance, x, y);
        intake.init(hwMap);
        arm.init(hwMap);
//        blinkin.init(hwMap);
    }
}
